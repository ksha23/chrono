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
// hash can use (those of numCells - 1). If that bound were too small, two occupied cells whose hashes
// agree in the sorted bits would get interleaved markers, their cell ranges would be wrong, and the
// markers would miss neighbors.
//
// The test runs the same fluid cube (added in a scrambled order) twice: in a computational domain barely
// larger than the cube, so that nearly every cell is occupied and any truncation of the key range makes
// occupied cells collide,
// and in the middle of a much larger domain, with a different grid and hash range. With no gravity
// and no walls near the fluid, the two runs must agree to round-off (positions differ by the change of
// grid origin, which perturbs the last bits of the relative distances).
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
const int n = 20;  // particles per side of the fluid cube

// Run the fluid cube in a domain spanning [c_min, c_max] and return the velocities of the fluid particles.
std::vector<ChVector3d> RunCube(const ChVector3d& c_min, const ChVector3d& c_max, int num_steps) {
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

    // A cube with a shear velocity field, so that particles move relative to each other. The particles are added in
    // a scrambled order: the sort breaks ties between equal keys by marker index, so with a lexicographic order the
    // markers of two cells whose truncated keys collide would still come out contiguous and the defect would hide.
    int num = n * n * n;
    for (int m = 0; m < num; m++) {
        int idx = (int)(((long long)m * 7919) % num);  // 7919 is prime and does not divide num, so this is a permutation
        int i = idx / (n * n);
        int j = (idx / n) % n;
        int k = idx % n;
        sysSPH.AddSPHParticle(ChVector3d(i * spacing, j * spacing, k * spacing), fluid_props.density, 0.0, fluid_props.viscosity, ChVector3d(0.05 * k / n, 0, 0));
    }

    sysSPH.SetComputationalDomain(ChAABB(c_min, c_max));
    sysSPH.ChFsiFluidSystem::Initialize();  // the base no-arg overload is hidden

    for (int step = 0; step < num_steps; step++)
        sysSPH.DoStepDynamics(dt);

    return sysSPH.GetParticleVelocities();
}

}  // namespace

TEST(SPHCellSort, DenseAndSparseGrids) {
    double size = (n - 1) * spacing;

    // Tight domain (about 10 x 10 x 10 cells of size 2h = 0.024 m, nearly all occupied) and a large one
    // (about 70 x 70 x 70 cells, 19 bits of hash)
    auto v_tight = RunCube(ChVector3d(-0.02), ChVector3d(size + 0.02), 30);
    auto v_large = RunCube(ChVector3d(-0.8), ChVector3d(size + 0.8), 30);

    ASSERT_EQ(v_tight.size(), (size_t)(n * n * n));
    ASSERT_EQ(v_large.size(), v_tight.size());

    double vmax = 0;
    double max_diff = 0;
    for (size_t i = 0; i < v_tight.size(); i++) {
        vmax = std::max(vmax, v_large[i].Length());
        max_diff = std::max(max_diff, (v_tight[i] - v_large[i]).Length());
    }

    // The flow is not trivial, and the two runs agree. Markers that missed neighbors would change the velocities by
    // a sizeable fraction of the velocity scale, not by round-off.
    EXPECT_GT(vmax, 1e-3);
    EXPECT_LT(max_diff, 1e-4 * vmax) << "max velocity difference " << max_diff << " (velocity scale " << vmax << ")";
}
