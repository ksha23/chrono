// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Unit test for the GPU error checks of the SPH fluid solver.
//
// The per-kernel checks (non-finite particle state, boundary condition and right-hand side
// failures, rheology failure) are collected in device-side flags and reported once per step, at
// the beginning of the next step, instead of synchronizing with the device after each kernel. This
// test pins the observable behavior:
//
//   1. A particle state that becomes non-finite is still reported by throwing, no later than the
//      step after the one in which it appeared. A single particle is given a NaN velocity, so its
//      state becomes NaN in the first step. The neighbor search runs only every 1000 steps, so the
//      proximity kernels (whose position check still reports immediately) never see the NaN and the
//      report must come from the deferred kernel checks.
//   2. With error checking disabled, the same run is not aborted by those checks.
//   3. Error checking does not change the results: a clean run gives bitwise identical particle
//      states with checking enabled and disabled.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

namespace {

const double spacing = 0.01;
const double dt = 1e-4;

// Create a small, standalone CFD fluid block (no walls, no gravity). If requested, the particle closest
// to the block center gets a NaN velocity.
void CreateFluid(ChFsiFluidSystemSPH& sysSPH, bool check_errors, bool poison) {
    sysSPH.SetVerbose(false);
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.num_proximity_search_steps = 1000;
    sysSPH.SetSPHParameters(sph_params);
    sysSPH.SetStepSize(dt);
    sysSPH.EnableGPUErrorCheck(check_errors);

    int n = 8;
    double nan = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                ChVector3d pos(i * spacing, j * spacing, k * spacing);
                bool center = (i == n / 2 && j == n / 2 && k == n / 2);
                ChVector3d vel = (poison && center) ? ChVector3d(nan, 0, 0) : ChVector3d(0.01 * i, 0, -0.01 * k);
                sysSPH.AddSPHParticle(pos, fluid_props.density, 0.0, fluid_props.viscosity, vel);
            }
        }
    }

    ChVector3d c_min(-0.5, -0.5, -0.5);
    ChVector3d c_max(+0.5, +0.5, +0.5);
    sysSPH.SetComputationalDomain(ChAABB(c_min, c_max));

    sysSPH.ChFsiFluidSystem::Initialize();  // the base no-arg overload is hidden
}

}  // namespace

// A non-finite particle state is reported no later than the step after the one in which it appears.
TEST(SPHErrorCheck, NonFiniteStateIsReported) {
    ChFsiFluidSystemSPH sysSPH;
    CreateFluid(sysSPH, true, true);

    int thrown_at = -1;
    std::string message;
    for (int step = 0; step < 3 && thrown_at < 0; step++) {
        try {
            sysSPH.DoStepDynamics(dt);
        } catch (const std::exception& e) {
            thrown_at = step;
            message = e.what();
        }
    }

    // The first kernel to flag the NaN is the right-hand side evaluation (a NaN velocity makes the derivatives of
    // the particle and its neighbors non-finite), so the message names that kernel, not a specific quantity.
    ASSERT_GE(thrown_at, 0) << "the non-finite particle state was not reported";
    EXPECT_LE(thrown_at, 1) << "the non-finite particle state was reported late";
    std::cout << "reported at step " << thrown_at << ": " << message << std::endl;
}

// With error checking disabled, the integration-step checks do not abort the run.
TEST(SPHErrorCheck, DisabledChecksDoNotThrow) {
    ChFsiFluidSystemSPH sysSPH;
    CreateFluid(sysSPH, false, true);

    EXPECT_NO_THROW({
        for (int step = 0; step < 3; step++)
            sysSPH.DoStepDynamics(dt);
    });
}

// Error checking does not change the results of a clean run.
TEST(SPHErrorCheck, ChecksDoNotChangeResults) {
    std::vector<ChVector3d> pos[2];
    std::vector<ChVector3d> vel[2];
    for (int c = 0; c < 2; c++) {
        ChFsiFluidSystemSPH sysSPH;
        CreateFluid(sysSPH, c == 1, false);
        for (int step = 0; step < 20; step++)
            sysSPH.DoStepDynamics(dt);
        pos[c] = sysSPH.GetParticlePositions();
        vel[c] = sysSPH.GetParticleVelocities();
    }

    ASSERT_EQ(pos[0].size(), pos[1].size());
    size_t num_diff = 0;
    for (size_t i = 0; i < pos[0].size(); i++) {
        if (!(pos[0][i] == pos[1][i]) || !(vel[0][i] == vel[1][i]))
            num_diff++;
    }
    EXPECT_EQ(num_diff, 0u);
}
