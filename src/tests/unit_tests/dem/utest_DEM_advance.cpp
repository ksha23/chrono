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
// Advancing a DEM system by several steps in a single call to AdvanceSimulation
// must give the same result as advancing it one step per call. In the first
// case the steps are queued on the device back to back; in the second case the
// host waits for the device after every step. A layer of spheres settles on
// the bottom of the box (and, optionally, on a force-tracking plane boundary
// condition). The mesh system variants also carry a static mesh. Results may
// differ in the last bits because force accumulation with atomics is not
// ordered, hence the tolerances.
//
// =============================================================================

#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_dem/physics/ChSystemDem.h"

using namespace chrono;
using namespace chrono::dem;

struct AdvanceResult {
    std::vector<ChVector3f> pos;
    ChVector3f bc_force;
};

static AdvanceResult Run(bool mesh_system, bool with_bc, int num_steps, bool one_call) {
    const float radius = 0.5f;
    const float step_size = 1e-4f;

    const ChVector3f box(20.f, 20.f, 20.f);
    std::unique_ptr<ChSystemDem> sys(mesh_system ? new ChSystemDemMesh(radius, 1.5f, box) : new ChSystemDem(radius, 1.5f, box));

    sys->SetGravitationalAcceleration(ChVector3f(0, 0, -980.f));
    sys->SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    sys->SetKn_SPH2SPH(1e7);
    sys->SetKn_SPH2WALL(1e7);
    sys->SetGn_SPH2SPH(2e4);
    sys->SetGn_SPH2WALL(2e4);
    sys->SetKt_SPH2SPH(2e6);
    sys->SetKt_SPH2WALL(1e6);
    sys->SetGt_SPH2SPH(50);
    sys->SetGt_SPH2WALL(50);
    sys->SetStaticFrictionCoeff_SPH2SPH(0.5f);
    sys->SetStaticFrictionCoeff_SPH2WALL(0.5f);
    sys->SetFixedStepSize(step_size);
    sys->SetBDFixed(true);

    // Two layers of 8 x 8 spheres, slightly separated, above the bottom of the box (or of the plane BC).
    float bottom = with_bc ? -5.f : -10.f;
    std::vector<ChVector3f> points;
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 8; j++)
            for (int i = 0; i < 8; i++)
                points.push_back(ChVector3f(-4.f + 1.02f * i + 0.01f * k, -4.f + 1.02f * j, bottom + radius + 0.05f + 1.05f * k));
    sys->SetParticles(points);

    size_t bc_id = 0;
    if (with_bc)
        bc_id = sys->CreateBCPlane(ChVector3f(0, 0, bottom), ChVector3f(0, 0, 1), true);

    // With the mesh system, add a static square (two triangles) well above the spheres, so that the triangle
    // broadphase runs at every step.
    auto msys = dynamic_cast<ChSystemDemMesh*>(sys.get());
    if (msys) {
        auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mesh->AddTriangle(ChVector3d(-1, -1, 5), ChVector3d(1, -1, 5), ChVector3d(1, 1, 5));
        mesh->AddTriangle(ChVector3d(-1, -1, 5), ChVector3d(1, 1, 5), ChVector3d(-1, 1, 5));
        msys->AddMesh(mesh, 1.f);
        msys->EnableMeshCollision(true);
        // Set explicitly: otherwise the uninitialized mesh contact model flag may be read when computing units.
        msys->UseMaterialBasedModel(false);
    }

    sys->Initialize();
    if (msys)
        msys->ApplyMeshMotion(0, ChVector3d(0, 0, 0), QUNIT, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));

    if (one_call) {
        sys->AdvanceSimulation(num_steps * step_size);
    } else {
        for (int n = 0; n < num_steps; n++)
            sys->AdvanceSimulation(step_size);
    }

    AdvanceResult res;
    for (int i = 0; i < (int)points.size(); i++)
        res.pos.push_back(sys->GetParticlePosition(i));
    res.bc_force = ChVector3f(0);
    if (with_bc)
        sys->GetBCReactionForces(bc_id, res.bc_force);
    return res;
}

static void Compare(bool mesh_system, bool with_bc) {
    const int num_steps = 500;
    auto a = Run(mesh_system, with_bc, num_steps, false);
    auto b = Run(mesh_system, with_bc, num_steps, true);

    ASSERT_EQ(a.pos.size(), b.pos.size());
    for (size_t i = 0; i < a.pos.size(); i++) {
        EXPECT_NEAR(a.pos[i].x(), b.pos[i].x(), 1e-3);
        EXPECT_NEAR(a.pos[i].y(), b.pos[i].y(), 1e-3);
        EXPECT_NEAR(a.pos[i].z(), b.pos[i].z(), 1e-3);
    }

    // The spheres fell and did not escape the box or pass through the plane.
    float bottom = with_bc ? -5.f : -10.f;
    for (const auto& p : b.pos) {
        EXPECT_GT(p.z(), bottom);
        EXPECT_LT(p.z(), bottom + 3.f);
    }

    if (with_bc) {
        // After 0.05 s the spheres rest on the plane, which carries their weight (the reaction force is the force
        // exerted by the spheres on the plane).
        double weight = a.pos.size() * (4.0 / 3.0) * CH_PI * 0.125 * 1.5 * 980;
        EXPECT_NEAR(-b.bc_force.z(), weight, 0.1 * weight);
        EXPECT_NEAR(a.bc_force.z(), b.bc_force.z(), 1e-2 * std::abs(a.bc_force.z()));
    }
}

TEST(demAdvance, system) {
    Compare(false, false);
}

TEST(demAdvance, mesh_system) {
    Compare(true, false);
}

TEST(demAdvance, mesh_system_bc) {
    Compare(true, true);
}
