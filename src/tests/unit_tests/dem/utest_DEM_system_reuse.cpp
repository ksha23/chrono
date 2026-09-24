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
// Create, advance and destroy several DEM systems, one after the other, in the
// same process. Each system must start from its own default parameters, not
// from values left in (reused) managed memory by an earlier system, and a mesh
// system without meshes must be destroyed cleanly.
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

// Let two layers of spheres settle on the bottom of the box (user-defined contact model, the default) and return the
// elapsed time reported by AdvanceSimulation and the mean height change. With the mesh system, optionally add a static
// mesh.
static void RunLayers(bool mesh_system, bool add_mesh, double& elapsed, double& dz) {
    const float radius = 0.5f;
    const float step_size = 1e-4f;
    const int num_steps = 500;

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
    sys->SetVerbosity(CHDEM_VERBOSITY::QUIET);

    std::vector<ChVector3f> points;
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 8; j++)
            for (int i = 0; i < 8; i++)
                points.push_back(ChVector3f(-4.f + 1.02f * i + 0.01f * k, -4.f + 1.02f * j, -10.f + radius + 0.05f + 1.05f * k));
    sys->SetParticles(points);

    auto msys = dynamic_cast<ChSystemDemMesh*>(sys.get());
    if (msys && add_mesh) {
        auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mesh->AddTriangle(ChVector3d(-1, -1, 5), ChVector3d(1, -1, 5), ChVector3d(1, 1, 5));
        mesh->AddTriangle(ChVector3d(-1, -1, 5), ChVector3d(1, 1, 5), ChVector3d(-1, 1, 5));
        msys->AddMesh(mesh, 1.f);
        msys->EnableMeshCollision(true);
    }

    sys->Initialize();
    if (msys && add_mesh)
        msys->ApplyMeshMotion(0, ChVector3d(0, 0, 0), QUNIT, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));

    elapsed = 0;
    for (int n = 0; n < num_steps; n++)
        elapsed += sys->AdvanceSimulation(step_size);

    dz = 0;
    for (int i = 0; i < (int)points.size(); i++)
        dz += (sys->GetParticlePosition(i).z() - points[i].z()) / points.size();
}

static void Check(bool mesh_system, bool add_mesh) {
    double elapsed, dz;
    RunLayers(mesh_system, add_mesh, elapsed, dz);
    // 500 steps of 1e-4 s; the two layers drop by about 0.05 and 0.1 onto the bottom of the box
    EXPECT_NEAR(elapsed, 0.05, 1e-6);
    EXPECT_NEAR(dz, -0.075, 0.015);
}

// A mesh system (with a mesh) created after a sphere-only system.
TEST(demSystemReuse, mesh_after_system) {
    Check(false, false);
    Check(true, true);
}

// A mesh system without meshes, created after a sphere-only system, and destroyed.
TEST(demSystemReuse, no_mesh_after_system) {
    Check(false, false);
    Check(true, false);
}
