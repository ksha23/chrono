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
// Unit test: fixed and moving boundary conditions (a sphere resting on a fixed plane, a plane with a prescribed
// motion, moving big-domain walls, and the reaction force on a plane that tracks forces)
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <memory>

#include "chrono_dem/physics/ChSystemDem.h"

using namespace chrono;
using namespace chrono::dem;

static const float radius = 0.5f;
static const float density = 2.5f;
static const float g = 980.0f;
static const float step_size = 1e-5f;

// A single sphere in a 20x20x20 box, at rest at the given height
static std::unique_ptr<ChSystemDem> CreateSystem(float z0) {
    auto sys = std::make_unique<ChSystemDem>(radius, density, ChVector3f(20.0f, 20.0f, 20.0f));
    sys->SetParticles(std::vector<ChVector3f>{ChVector3f(0.0f, 0.0f, z0)});
    sys->SetPsiFactors(32, 16);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    sys->SetKn_SPH2SPH(1e7);
    sys->SetKn_SPH2WALL(1e7);
    sys->SetGn_SPH2SPH(1e4);
    sys->SetGn_SPH2WALL(1e4);
    sys->SetKt_SPH2SPH(1e7);
    sys->SetKt_SPH2WALL(1e7);
    sys->SetGt_SPH2SPH(1e3);
    sys->SetGt_SPH2WALL(1e3);
    sys->SetStaticFrictionCoeff_SPH2SPH(0.5f);
    sys->SetStaticFrictionCoeff_SPH2WALL(0.5f);
    sys->SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    sys->SetFixedStepSize(step_size);
    sys->SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    sys->SetBDFixed(true);
    sys->SetVerbosity(CHDEM_VERBOSITY::QUIET);
    return sys;
}

// A sphere dropped on a fixed plane comes to rest on it
TEST(demBoundaries, fixed_plane) {
    float plane_z = -5.0f;
    auto sys = CreateSystem(plane_z + 2 * radius);
    sys->CreateBCPlane(ChVector3f(0, 0, plane_z), ChVector3f(0, 0, 1), false);
    sys->Initialize();

    sys->AdvanceSimulation(0.2f);

    auto pos = sys->GetParticlePosition(0);
    auto vel = sys->GetParticleVelocity(0);
    std::cout << "fixed plane: z = " << pos.z() << " vz = " << vel.z() << std::endl;
    ASSERT_NEAR(pos.z(), plane_z + radius, 0.05 * radius);
    ASSERT_NEAR(pos.x(), 0.0, 1e-3);
    ASSERT_NEAR(pos.y(), 0.0, 1e-3);
    ASSERT_NEAR(vel.z(), 0.0, 1.0);
}

// A plane moving up at constant speed carries a sphere resting on it
TEST(demBoundaries, moving_plane) {
    float plane_z = -5.0f;
    float speed = 20.0f;
    auto sys = CreateSystem(plane_z + radius);
    size_t id = sys->CreateBCPlane(ChVector3f(0, 0, plane_z), ChVector3f(0, 0, 1), false);
    sys->Initialize();
    sys->SetBCOffsetFunction(id, [speed](float t) { return make_double3(0, 0, speed * t); });

    float duration = 0.2f;
    sys->AdvanceSimulation(duration);

    auto pos = sys->GetParticlePosition(0);
    auto vel = sys->GetParticleVelocity(0);
    std::cout << "moving plane: z = " << pos.z() << " vz = " << vel.z() << std::endl;
    ASSERT_NEAR(pos.z(), plane_z + speed * duration + radius, 0.05 * radius);
    ASSERT_NEAR(vel.z(), speed, 0.05 * speed);
}

// Big-domain walls with a prescribed motion carry a sphere resting on the bottom wall
TEST(demBoundaries, moving_domain_walls) {
    float floor_z = -10.0f;
    float speed = 20.0f;
    auto sys = CreateSystem(floor_z + radius);
    sys->Initialize();
    sys->setBDWallsMotionFunction([speed](float t) { return make_double3(0, 0, speed * t); });

    float duration = 0.2f;
    sys->AdvanceSimulation(duration);

    auto pos = sys->GetParticlePosition(0);
    auto vel = sys->GetParticleVelocity(0);
    std::cout << "moving domain walls: z = " << pos.z() << " vz = " << vel.z() << std::endl;
    ASSERT_NEAR(pos.z(), floor_z + speed * duration + radius, 0.05 * radius);
    ASSERT_NEAR(vel.z(), speed, 0.05 * speed);
}

// The reaction force on a plane that tracks forces balances the weight of the sphere resting on it, at every call
TEST(demBoundaries, reaction_force) {
    float plane_z = -5.0f;
    auto sys = CreateSystem(plane_z + radius);
    size_t id = sys->CreateBCPlane(ChVector3f(0, 0, plane_z), ChVector3f(0, 0, 1), true);
    sys->Initialize();

    sys->AdvanceSimulation(0.2f);

    double weight = (4.0 / 3.0) * CH_PI * radius * radius * radius * density * g;
    for (int i = 0; i < 3; i++) {
        sys->AdvanceSimulation(0.01f);
        ChVector3f force;
        ASSERT_TRUE(sys->GetBCReactionForces(id, force));
        std::cout << "reaction force: " << force << "  weight: " << weight << std::endl;
        ASSERT_NEAR(std::abs(force.z()), weight, 0.02 * weight);
        ASSERT_NEAR(force.x(), 0.0, 1e-3 * weight);
        ASSERT_NEAR(force.y(), 0.0, 1e-3 * weight);
    }
}
