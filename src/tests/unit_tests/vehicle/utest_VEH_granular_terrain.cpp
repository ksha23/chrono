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
// Test that GranularTerrain::GetHeight and GetPoint report the terrain below the
// query location and not the highest particle over the whole patch.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

class GranularTerrainHeight : public ::testing::Test {
  protected:
    GranularTerrainHeight() : terrain(&sys) {
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        terrain.SetContactMaterial(chrono_types::make_shared<ChContactMaterialSMC>());
        terrain.Initialize(ChVector3d(0, 0, 0), length, width, num_layers, radius, 2000.0);
    }

    // Visit all granular particles (all bodies other than the terrain ground body).
    template <typename F>
    void ForEachParticle(F f) {
        for (const auto& body : sys.GetBodies()) {
            if (body != terrain.GetGroundBody())
                f(body);
        }
    }

    // Height of the top of the highest particle whose center is in the given X range.
    double MaxTop(double xmin, double xmax) {
        double h = -1e10;
        ForEachParticle([&](const std::shared_ptr<ChBody>& b) {
            if (b->GetPos().x() > xmin && b->GetPos().x() < xmax)
                h = std::max(h, b->GetPos().z() + radius);
        });
        return h;
    }

    const double length = 1.0;
    const double width = 0.4;
    const unsigned int num_layers = 3;
    const double radius = 0.02;

    ChSystemSMC sys;
    GranularTerrain terrain;
};

// Raise all particles on the +X half of the patch to form a mound. The height reported above the mound must exceed
// the height reported above the flat half by the mound height (up to one particle diameter of sampling noise).
TEST_F(GranularTerrainHeight, Mound) {
    const double mound = 0.2;
    ForEachParticle([&](const std::shared_ptr<ChBody>& b) {
        if (b->GetPos().x() > 0)
            b->SetPos(b->GetPos() + ChVector3d(0, 0, mound));
    });

    double h_flat = terrain.GetHeight(ChVector3d(-0.25, 0, 0));
    double h_mound = terrain.GetHeight(ChVector3d(+0.25, 0, 0));

    EXPECT_LE(h_flat, MaxTop(-1, 0) + 1e-12);
    EXPECT_LE(h_mound, MaxTop(0, 1) + 1e-12);
    EXPECT_NEAR(h_mound - h_flat, mound, 2 * radius);
}

// Directly above a particle center, the height is at least the top of that particle; the returned point lies on the
// vertical line through the query location.
TEST_F(GranularTerrainHeight, AboveParticle) {
    ForEachParticle([&](const std::shared_ptr<ChBody>& b) {
        ChVector3d loc(b->GetPos().x(), b->GetPos().y(), 10.0);
        auto p = terrain.GetPoint(loc);
        EXPECT_EQ(p.x(), loc.x());
        EXPECT_EQ(p.y(), loc.y());
        EXPECT_GE(p.z(), b->GetPos().z() + radius - 1e-12);
        EXPECT_DOUBLE_EQ(terrain.GetHeight(loc), p.z());
    });
}

// Outside the patch, no particle is hit and the height is that of the bottom boundary.
TEST_F(GranularTerrainHeight, OutsidePatch) {
    EXPECT_DOUBLE_EQ(terrain.GetHeight(ChVector3d(10 * length, 0, 0)), terrain.GetPatchBottom());
}
