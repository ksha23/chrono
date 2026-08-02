// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Tests for ChSolverAuto's dispatch rule and for the iterative VI solvers'
// stopping criterion.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChSolverAuto.h"
#include "chrono/solver/ChSolverBB.h"

using namespace chrono;

namespace {

// A short chain of revolute joints: bilateral only, no contacts.
void BuildChain(ChSystemNSC& sys, int n) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    std::shared_ptr<ChBody> prev = ground;
    for (int i = 0; i < n; i++) {
        auto b = chrono_types::make_shared<ChBody>();
        b->SetMass(1.0);
        b->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
        b->SetPos(ChVector3d(0.5 + i, 0, 0));
        sys.AddBody(b);
        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(prev, b, ChFrame<>(ChVector3d(i, 0, 0), QUNIT));
        sys.AddLink(rev);
        prev = b;
    }
}

// A ball resting on a fixed plate: generates NSC contacts.
// NOTE: the collision system type must be set explicitly, otherwise no contact is
// ever generated and a test like this silently passes for the wrong reason.
void BuildContact(ChSystemNSC& sys, double drop_height = 0.0) {
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetRestitution(0);

    double radius = 0.2;

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(mat, 4.0, 4.0, 0.2),
                              ChFrame<>(ChVector3d(0, 0, -0.1), QUNIT));
    sys.AddBody(ground);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(5);
    ball->SetInertiaXX(0.4 * 5 * radius * radius * ChVector3d(1, 1, 1));
    ball->SetPos(ChVector3d(0, 0, radius + 1e-3 + drop_height));
    ball->EnableCollision(true);
    ball->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(mat, radius));
    sys.AddBody(ball);
}

}  // namespace

// The dispatch rule: a bilateral-only model takes the direct path.
TEST(ChSolverAuto, BilateralModelUsesDirectSolver) {
    ChSystemNSC sys;
    BuildChain(sys, 6);
    sys.SetSolverType(ChSolver::Type::AUTO);

    for (int i = 0; i < 10; i++)
        sys.DoStepDynamics(1e-3);

    auto solver = std::dynamic_pointer_cast<ChSolverAuto>(sys.GetSolver());
    ASSERT_NE(solver, nullptr);
    EXPECT_TRUE(solver->UsedDirectSolver());
    EXPECT_EQ(solver->GetReason(), ChSolverAuto::Reason::DIRECT);
}

// A model with contacts must NOT take the direct path: a direct linear solver
// would treat every contact as bilateral glue.
TEST(ChSolverAuto, ContactModelUsesIterativeSolver) {
    ChSystemNSC sys;
    BuildContact(sys);
    sys.SetSolverType(ChSolver::Type::AUTO);

    bool saw_contact = false;
    auto solver = std::dynamic_pointer_cast<ChSolverAuto>(sys.GetSolver());
    ASSERT_NE(solver, nullptr);

    for (int i = 0; i < 60; i++) {
        sys.DoStepDynamics(1e-3);
        if (sys.GetNumContacts() > 0) {
            saw_contact = true;
            // While contacts are live, the direct solver must never be selected.
            EXPECT_FALSE(solver->UsedDirectSolver());
            EXPECT_EQ(solver->GetReason(), ChSolverAuto::Reason::UNILATERAL);
        }
    }
    EXPECT_TRUE(saw_contact) << "test model never produced a contact";
}

// A model that acquires contacts part-way through must switch away from the
// direct solver rather than keep using it.
TEST(ChSolverAuto, SwitchesWhenContactAppears) {
    ChSystemNSC sys;
    BuildContact(sys, 1.0);  // dropped from a height, so the first steps are contact-free
    sys.SetSolverType(ChSolver::Type::AUTO);
    auto solver = std::dynamic_pointer_cast<ChSolverAuto>(sys.GetSolver());
    ASSERT_NE(solver, nullptr);

    sys.DoStepDynamics(1e-3);
    ASSERT_EQ(sys.GetNumContacts(), 0u) << "model was not contact-free at the start";
    EXPECT_TRUE(solver->UsedDirectSolver()) << "free flight should use the direct solver";

    bool switched = false;
    for (int i = 0; i < 2000 && !switched; i++) {
        sys.DoStepDynamics(1e-3);
        if (sys.GetNumContacts() > 0 && !solver->UsedDirectSolver())
            switched = true;
    }
    EXPECT_TRUE(switched) << "solver did not switch away from the direct path on contact";
}

// The direct path must reproduce a plain sparse-LU solve exactly.
TEST(ChSolverAuto, MatchesDirectSolverOnBilateralModel) {
    std::vector<ChVector3d> ref;
    {
        ChSystemNSC sys;
        BuildChain(sys, 8);
        sys.SetSolverType(ChSolver::Type::SPARSE_LU);
        sys.GetSolver()->AsDirect()->LockSparsityPattern(true);
        for (int i = 0; i < 200; i++) {
            sys.DoStepDynamics(1e-3);
            ref.push_back(sys.GetBodies().back()->GetPos());
        }
    }

    ChSystemNSC sys;
    BuildChain(sys, 8);
    sys.SetSolverType(ChSolver::Type::AUTO);
    for (size_t i = 0; i < ref.size(); i++) {
        sys.DoStepDynamics(1e-3);
        auto p = sys.GetBodies().back()->GetPos();
        ASSERT_LT((p - ref[i]).Length(), 1e-12) << "diverged from the direct solve at step " << i;
    }
}

// The user override must be honoured, so a previous result can be reproduced.
TEST(ChSolverAuto, ForceIterativeIsHonoured) {
    ChSystemNSC sys;
    BuildChain(sys, 6);
    sys.SetSolverType(ChSolver::Type::AUTO);
    auto solver = std::dynamic_pointer_cast<ChSolverAuto>(sys.GetSolver());
    ASSERT_NE(solver, nullptr);
    solver->ForceIterative(true);

    for (int i = 0; i < 10; i++)
        sys.DoStepDynamics(1e-3);

    EXPECT_FALSE(solver->UsedDirectSolver());
}

// Existing calling code configures the iterative fallback through AsIterative().
TEST(ChSolverAuto, AsIterativeConfiguresTheFallback) {
    ChSystemNSC sys;
    sys.SetSolverType(ChSolver::Type::AUTO);
    ASSERT_NE(sys.GetSolver()->AsIterative(), nullptr);

    sys.GetSolver()->AsIterative()->SetMaxIterations(37);
    auto solver = std::dynamic_pointer_cast<ChSolverAuto>(sys.GetSolver());
    ASSERT_NE(solver, nullptr);
    EXPECT_EQ(solver->GetIterativeSolver()->GetMaxIterations(), 37);
}

// Regression guard for the defect this work started from: the VI solvers'
// stopping test must be reachable. A zero tolerance makes `residual < tol`
// unsatisfiable, so the solver can never exit before its iteration budget.
TEST(ChIterativeSolverVI, DefaultToleranceIsReachable) {
    for (auto type : {ChSolver::Type::PSOR, ChSolver::Type::PJACOBI, ChSolver::Type::BARZILAIBORWEIN,
                      ChSolver::Type::APGD}) {
        ChSystemNSC sys;
        sys.SetSolverType(type);
        ASSERT_NE(sys.GetSolver()->AsIterative(), nullptr);
        EXPECT_GT(sys.GetSolver()->AsIterative()->GetTolerance(), 0.0)
            << ChSolver::GetTypeAsString(type) << " has an unsatisfiable stopping criterion";
    }
}

// On a well-conditioned problem the solver must actually stop early rather than
// burn its whole budget.
TEST(ChIterativeSolverVI, ExitsEarlyOnConvergedProblem) {
    ChSystemNSC sys;
    BuildChain(sys, 1);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    auto solver = sys.GetSolver()->AsIterative();
    solver->SetMaxIterations(500);

    int n_early = 0;
    for (int i = 0; i < 50; i++) {
        sys.DoStepDynamics(1e-3);
        if (solver->GetIterations() < solver->GetMaxIterations())
            n_early++;
    }
    EXPECT_GT(n_early, 0) << "solver never exited before its iteration budget";
}

// The repaired Armijo line search is off by default, so existing BB results are
// unchanged. Guard against it being switched on inadvertently.
TEST(ChSolverBB, ArmijoLineSearchIsOffByDefault) {
    ChSolverBB solver;
    EXPECT_EQ(solver.GetMaxStepsArmijoBacktrace(), 0);
}
