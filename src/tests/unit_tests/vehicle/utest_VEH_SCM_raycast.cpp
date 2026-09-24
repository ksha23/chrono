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
// Test that the SCM CPU ray-casting results do not depend on the number of
// OpenMP threads: ray hits, contact patches, terrain deformation, contact forces
// and body motion must be identical (bitwise) to the single-threaded run.
//
// Two wheels (cylinders) roll on SCM terrain. One wheel is tracked by two
// overlapping active domains, so that some grid nodes belong to both domains.
//
// =============================================================================

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

struct StepData {
    int num_ray_casts;
    int num_ray_hits;
    int num_patches;
    ChVector3d force;
    ChVector3d torque;
};

struct RunData {
    std::vector<StepData> steps;
    std::vector<SCMTerrain::NodeLevel> nodes;  // all modified nodes, sorted by grid index
    std::vector<ChVector3d> pos;               // final body positions
    std::vector<ChVector3d> vel;               // final body velocities
    std::vector<ChVector3d> angvel;            // final body angular velocities
};

static RunData RunSCM(int num_threads, int num_steps) {
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetNumThreads(num_threads, 1, 1);

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();

    std::vector<std::shared_ptr<ChBody>> wheels;
    for (int i = 0; i < 2; i++) {
        auto wheel = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.3, 0.2, 500, true, true, mat);
        wheel->SetPos(ChVector3d(-0.5, i == 0 ? -0.5 : 0.5, 0.29));
        wheel->SetPosDt(ChVector3d(1.0 + 0.5 * i, 0, 0));
        wheel->SetAngVelParent(ChVector3d(0, 2.0 + 1.0 * i, 0));
        sys.AddBody(wheel);
        wheels.push_back(wheel);
    }

    SCMTerrain terrain(&sys, false);
    terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4);
#ifdef CHRONO_HAS_SCM_GPU
    // Exercise the CPU ray-casting and contact-force paths
    terrain.EnableRaycastGpuHip(false);
    terrain.SetScmGpuEnabled(false);
#endif
    terrain.AddActiveDomain(wheels[0], VNULL, ChVector3d(0.8, 0.4, 0.8));
    terrain.AddActiveDomain(wheels[1], VNULL, ChVector3d(0.8, 0.4, 0.8));
    terrain.AddActiveDomain(wheels[1], ChVector3d(0.2, 0, 0), ChVector3d(0.8, 0.4, 0.8));
    terrain.Initialize(4.0, 3.0, 0.02);

    RunData data;
    double step_size = 1e-3;
    for (int k = 0; k < num_steps; k++) {
        sys.DoStepDynamics(step_size);
        StepData sd;
        sd.num_ray_casts = terrain.GetNumRayCasts();
        sd.num_ray_hits = terrain.GetNumRayHits();
        sd.num_patches = terrain.GetNumContactPatches();
        terrain.GetContactForceBody(wheels[1], sd.force, sd.torque);
        data.steps.push_back(sd);
    }

    data.nodes = terrain.GetModifiedNodes(true);
    std::sort(data.nodes.begin(), data.nodes.end(), [](const SCMTerrain::NodeLevel& a, const SCMTerrain::NodeLevel& b) {
        return a.first.x() < b.first.x() || (a.first.x() == b.first.x() && a.first.y() < b.first.y());
    });
    for (auto& w : wheels) {
        data.pos.push_back(w->GetPos());
        data.vel.push_back(w->GetPosDt());
        data.angvel.push_back(w->GetAngVelParent());
    }

    return data;
}

TEST(SCMTerrain, raycast_thread_invariance) {
    const int num_steps = 300;
    RunData ref = RunSCM(1, num_steps);

    // Sanity check: the wheels are in contact with the terrain and deform it
    ASSERT_GT(ref.steps.back().num_ray_hits, 0);
    ASSERT_GT(ref.steps.back().num_patches, 1);
    ASSERT_GT(ref.nodes.size(), 0);
    ASSERT_GT(ref.steps.back().force.z(), 0);

    for (int num_threads : {2, 3, 4}) {
        RunData crt = RunSCM(num_threads, num_steps);

        for (int k = 0; k < num_steps; k++) {
            ASSERT_EQ(crt.steps[k].num_ray_casts, ref.steps[k].num_ray_casts) << "threads " << num_threads << " step " << k;
            ASSERT_EQ(crt.steps[k].num_ray_hits, ref.steps[k].num_ray_hits) << "threads " << num_threads << " step " << k;
            ASSERT_EQ(crt.steps[k].num_patches, ref.steps[k].num_patches) << "threads " << num_threads << " step " << k;
            ASSERT_TRUE(crt.steps[k].force == ref.steps[k].force)
                << "threads " << num_threads << " step " << k << "  force " << crt.steps[k].force << " vs "
                << ref.steps[k].force;
            ASSERT_TRUE(crt.steps[k].torque == ref.steps[k].torque)
                << "threads " << num_threads << " step " << k << "  torque " << crt.steps[k].torque << " vs "
                << ref.steps[k].torque;
        }

        ASSERT_EQ(crt.nodes.size(), ref.nodes.size()) << "threads " << num_threads;
        for (size_t i = 0; i < ref.nodes.size(); i++) {
            ASSERT_EQ(crt.nodes[i].first, ref.nodes[i].first) << "threads " << num_threads;
            ASSERT_EQ(crt.nodes[i].second, ref.nodes[i].second) << "threads " << num_threads;
        }

        for (size_t i = 0; i < ref.pos.size(); i++) {
            ASSERT_TRUE(crt.pos[i] == ref.pos[i]) << "threads " << num_threads << "  " << crt.pos[i] << " vs " << ref.pos[i];
            ASSERT_TRUE(crt.vel[i] == ref.vel[i]) << "threads " << num_threads << "  " << crt.vel[i] << " vs " << ref.vel[i];
            ASSERT_TRUE(crt.angvel[i] == ref.angvel[i])
                << "threads " << num_threads << "  " << crt.angvel[i] << " vs " << ref.angvel[i];
        }
    }
}
