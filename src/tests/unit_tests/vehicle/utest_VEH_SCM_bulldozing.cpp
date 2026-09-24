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
// Authors: Chrono contributors
// =============================================================================
//
// Regression test for SCM bulldozing: a box is pushed through the soil with
// bulldozing enabled, and the resulting deformed height field is compared against
// reference values.
//
// =============================================================================

#include <cmath>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

TEST(SCMTerrain, bulldozing) {
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetNumThreads(1);

    SCMTerrain terrain(&sys, false);
    terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4);
    terrain.EnableBulldozing(true);
    terrain.SetBulldozingParameters(55, 1, 5, 6);
    terrain.Initialize(4.0, 2.0, 0.02);

    // Blade: a fixed box, lowered into the soil and dragged along +x
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto box = chrono_types::make_shared<ChBodyEasyBox>(0.3, 0.5, 0.2, 1000, false, true, mat);
    box->SetPos(ChVector3d(-1.0, 0.0, 0.1 - 0.01));
    box->SetFixed(true);
    sys.AddBody(box);

    const double step = 1e-3;
    int max_erosion_nodes = 0;
    for (int i = 0; i < 300; i++) {
        sys.DoStepDynamics(step);
        max_erosion_nodes = std::max(max_erosion_nodes, terrain.GetNumErosionNodes());
        double dz = (i < 50) ? 0.0006 : 0.0;                     // lower the blade 3 cm more over 50 steps
        box->SetPos(box->GetPos() + ChVector3d(0.004, 0, -dz));  // then drag it at 4 m/s
    }

    // Statistics of the deformed height field
    auto nodes = terrain.GetModifiedNodes(true);
    double sum = 0, sum2 = 0, zmin = 1e9, zmax = -1e9;
    for (const auto& n : nodes) {
        sum += n.second;
        sum2 += n.second * n.second;
        zmin = std::min(zmin, n.second);
        zmax = std::max(zmax, n.second);
    }
    std::cout.precision(17);
    std::cout << "nodes = " << nodes.size() << "  max erosion nodes = " << max_erosion_nodes << std::endl;
    std::cout << "sum = " << sum << "  sum2 = " << sum2 << "  zmin = " << zmin << "  zmax = " << zmax << std::endl;

    // Material is pushed below the blade and piled up next to the rut
    EXPECT_LT(zmin, -0.03);
    EXPECT_GT(zmax, 0.0);
    EXPECT_GT(max_erosion_nodes, 0);

    // Reference values (GCC 11.4, x86-64, 1 thread).
    // The erosion sweep updates nodes in place while iterating over an unordered set of grid nodes, so its result
    // depends on the iteration order of that set (e.g., on the hash function or the number of threads used for ray
    // casting). The tolerances below accept such a change of order (which moves the extreme levels by a few mm),
    // but not a change of the bulldozing algorithm itself.
    const double REF_NODES = 3537;
    const double REF_SUM = -58.505310557798552;
    const double REF_SUM2 = 2.5918938819942401;
    const double REF_ZMIN = -0.050178790182507566;
    const double REF_ZMAX = 0.058424501604116809;
    EXPECT_NEAR((double)nodes.size(), REF_NODES, 0.01 * REF_NODES);
    EXPECT_NEAR(sum, REF_SUM, 2e-3 * std::abs(REF_SUM));
    EXPECT_NEAR(sum2, REF_SUM2, 2e-2 * std::abs(REF_SUM2));
    EXPECT_NEAR(zmin, REF_ZMIN, 5e-3);
    EXPECT_NEAR(zmax, REF_ZMAX, 5e-3);
}
