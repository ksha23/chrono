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
// Unit test for the sort of SPH markers by grid cell.
//
// The proximity search sorts the markers by cell hash with a radix sort limited to the bits a valid
// hash can use (those of numCells - 1). If that bound were too small, markers in the cells with the
// largest hashes would be ordered by truncated keys, their cell ranges would be wrong, and they would
// miss neighbors.
//
// The test places the same small fluid block in a large computational domain twice: once in the
// cells with the smallest hashes (near the minimum corner) and once in the cells with the largest
// hashes (near the maximum corner). With no gravity and no walls the two blocks see the same relative
// geometry, so their velocity fields must agree to round-off (positions differ by the translation,
// which perturbs the last bits of the relative distances).
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

namespace {

const double spacing = 0.01;
const double dt = 1e-4;
const int n = 10;  // particles per side of the fluid block

// Run a fluid block whose lowest corner is at 'corner' in a domain spanning [c_min, c_max] and return the
// velocities of the fluid particles, in the order they were added.
std::vector<ChVector3d> RunBlock(const ChVector3d& corner, const ChVector3d& c_min, const ChVector3d& c_max, int num_steps) {
    ChFsiFluidSystemSPH sysSPH;
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
    sph_params.num_proximity_search_steps = 1;
    sysSPH.SetSPHParameters(sph_params);
    sysSPH.SetStepSize(dt);

    // A block with a shear velocity field, so that particles move relative to each other
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                sysSPH.AddSPHParticle(corner + ChVector3d(i * spacing, j * spacing, k * spacing), fluid_props.density, 0.0, fluid_props.viscosity,
                                      ChVector3d(0.05 * k * spacing / (n * spacing), 0, 0));

    sysSPH.SetComputationalDomain(ChAABB(c_min, c_max));
    sysSPH.ChFsiFluidSystem::Initialize();  // the base no-arg overload is hidden

    for (int step = 0; step < num_steps; step++)
        sysSPH.DoStepDynamics(dt);

    return sysSPH.GetParticleVelocities();
}

}  // namespace

TEST(SPHCellSort, BlockInHighestCellsMatchesBlockInLowestCells) {
    // Domain of 1.6 x 1.2 x 1.0 m; with a cell size of 2h = 0.024 m this is about 67 x 50 x 42 cells, so the
    // largest hash needs 18 bits.
    ChVector3d c_min(0, 0, 0);
    ChVector3d c_max(1.6, 1.2, 1.0);
    double block = (n - 1) * spacing;
    double margin = 0.05;

    auto v_low = RunBlock(c_min + ChVector3d(margin), c_min, c_max, 50);
    auto v_high = RunBlock(c_max - ChVector3d(margin + block), c_min, c_max, 50);

    ASSERT_EQ(v_low.size(), v_high.size());
    ASSERT_EQ(v_low.size(), (size_t)(n * n * n));

    double vmax = 0;
    double max_diff = 0;
    for (size_t i = 0; i < v_low.size(); i++) {
        vmax = std::max(vmax, v_low[i].Length());
        max_diff = std::max(max_diff, (v_low[i] - v_high[i]).Length());
    }

    // The flow is not trivial, and the two blocks evolve the same way. A block whose markers missed neighbors
    // would differ by a sizeable fraction of the velocity scale, not by round-off.
    EXPECT_GT(vmax, 1e-3);
    EXPECT_LT(max_diff, 1e-4 * vmax) << "max velocity difference " << max_diff << " (velocity scale " << vmax << ")";
}
