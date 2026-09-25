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
// Test that the implicit timesteppers report a failed linear solver setup
// (StateSolveCorrection returning false) instead of applying a stale correction,
// and that they never solve with, or refactorize, a matrix whose setup failed.
//
// =============================================================================

#include <stdexcept>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/solver/ChDirectSolverLS.h"

using namespace chrono;

// Direct solver stub (Eigen SparseLU) whose Setup can be forced to fail, e.g. to mimic a failed Pardiso analysis.
// It records any Solve() call without a valid factorization and any factorization without a valid analysis.
class StubDirectSolver : public ChDirectSolverLS {
  public:
    bool fail_always = false;  // fail every Setup
    bool fail_next = false;    // fail the next Setup only

    int num_setups = 0;             // number of FactorizeMatrix calls
    int num_failed_setups = 0;      // number of failed FactorizeMatrix calls
    int num_unfactored_solves = 0;  // SolveSystem calls after a failed factorization
    int num_unanalyzed_setups = 0;  // factorizations (analyze=false) after a failed analysis

  private:
    virtual bool FactorizeMatrix(bool analyze) override {
        num_setups++;
        if (!analyze && !m_analyzed)
            num_unanalyzed_setups++;

        if (fail_always || fail_next) {
            fail_next = false;
            num_failed_setups++;
            m_analyzed = false;
            m_factorized = false;
            return false;
        }

        m_engine.compute(m_mat);
        m_analyzed = true;
        m_factorized = (m_engine.info() == Eigen::Success);
        return m_factorized;
    }

    virtual bool SolveSystem() override {
        if (!m_factorized) {
            num_unfactored_solves++;
            m_sol.setZero();
            return false;
        }
        m_sol = m_engine.solve(m_rhs);
        return m_engine.info() == Eigen::Success;
    }

    virtual void PrintErrorMessage() override {}

    Eigen::SparseLU<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;
    bool m_analyzed = false;
    bool m_factorized = false;
};

// Simple pendulum (one body, one revolute joint) so that the linear system has both variables and constraints.
static std::shared_ptr<StubDirectSolver> CreatePendulum(ChSystemSMC& sys, ChTimestepper::Type type) {
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto pend = chrono_types::make_shared<ChBody>();
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    pend->SetPos(ChVector3d(1, 0, 0));
    sys.AddBody(pend);

    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    rev->Initialize(ground, pend, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sys.AddLink(rev);

    auto solver = chrono_types::make_shared<StubDirectSolver>();
    solver->UseSparsityPatternLearner(false);
    sys.SetSolver(solver);
    sys.SetTimestepperType(type);

    return solver;
}

class SolveFailure : public ::testing::TestWithParam<ChTimestepper::Type> {};

// Healthy solver: every implicit timestepper advances, with the stub never solving an unfactored matrix.
TEST_P(SolveFailure, healthy_solver) {
    ChSystemSMC sys;
    auto solver = CreatePendulum(sys, GetParam());

    for (int i = 0; i < 20; i++)
        ASSERT_NO_THROW(sys.DoStepDynamics(1e-3));

    EXPECT_GT(solver->num_setups, 0);
    EXPECT_EQ(solver->num_failed_setups, 0);
    EXPECT_EQ(solver->num_unfactored_solves, 0);
    EXPECT_LT(sys.GetBodies()[1]->GetPos().y(), 0);  // the pendulum started to fall
}

// Failing solver setup: the step must throw, and the stepper must not solve with the failed factorization.
TEST_P(SolveFailure, setup_fails) {
    ChSystemSMC sys;
    auto solver = CreatePendulum(sys, GetParam());
    solver->fail_always = true;

    EXPECT_THROW(sys.DoStepDynamics(1e-3), std::runtime_error);
    EXPECT_EQ(solver->num_failed_setups, 1);
    EXPECT_EQ(solver->num_unfactored_solves, 0);
}

// Transient setup failure after a few healthy steps: the exception must leave the system at the state and time of the
// beginning of the failed step, and a caller that catches it and steps again must reproduce the trajectory of a run
// without the failure.
TEST_P(SolveFailure, recover_after_setup_failure) {
    const double step = 1e-3;

    // Reference run without failure
    ChSystemSMC sys_ref;
    CreatePendulum(sys_ref, GetParam());
    for (int i = 0; i < 5; i++)
        ASSERT_NO_THROW(sys_ref.DoStepDynamics(step));
    auto pend_ref = sys_ref.GetBodies()[1];

    // Run with a failed setup at the third step
    ChSystemSMC sys;
    auto solver = CreatePendulum(sys, GetParam());
    auto pend = sys.GetBodies()[1];
    ASSERT_NO_THROW(sys.DoStepDynamics(step));
    ASSERT_NO_THROW(sys.DoStepDynamics(step));

    double time = sys.GetChTime();
    ChVector3d pos = pend->GetPos();
    ChQuaterniond rot = pend->GetRot();
    ChVector3d vel = pend->GetPosDt();
    ChVector3d angvel = pend->GetAngVelLocal();

    solver->fail_next = true;
    EXPECT_THROW(sys.DoStepDynamics(step), std::runtime_error);
    EXPECT_EQ(solver->num_failed_setups, 1);

    // State and time are those at the beginning of the failed step (the angular velocity up to the round-off of
    // converting it to and from the quaternion derivative when the state is gathered and scattered)
    EXPECT_EQ(sys.GetChTime(), time);
    EXPECT_EQ(pend->GetPos(), pos);
    EXPECT_EQ(pend->GetRot(), rot);
    EXPECT_EQ(pend->GetPosDt(), vel);
    EXPECT_NEAR((pend->GetAngVelLocal() - angvel).Length(), 0, 1e-15);

    // Stepping again reproduces the reference run
    for (int i = 0; i < 3; i++)
        ASSERT_NO_THROW(sys.DoStepDynamics(step));
    EXPECT_EQ(solver->num_unfactored_solves, 0);
    EXPECT_EQ(solver->num_unanalyzed_setups, 0);
    EXPECT_NEAR(sys.GetChTime(), sys_ref.GetChTime(), 1e-15);
    EXPECT_NEAR((pend->GetPos() - pend_ref->GetPos()).Length(), 0, 1e-10);
    EXPECT_NEAR((pend->GetPosDt() - pend_ref->GetPosDt()).Length(), 0, 1e-10);
}

INSTANTIATE_TEST_SUITE_P(CH_timestepper,
                         SolveFailure,
                         ::testing::Values(ChTimestepper::Type::EULER_IMPLICIT,
                                           ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED,
                                           ChTimestepper::Type::EULER_IMPLICIT_PROJECTED,
                                           ChTimestepper::Type::TRAPEZOIDAL,
                                           ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED,
                                           ChTimestepper::Type::NEWMARK,
                                           ChTimestepper::Type::HHT),
                         [](const ::testing::TestParamInfo<ChTimestepper::Type>& info) { return ChTimestepper::GetTypeAsString(info.param); });

// HHT with a transient setup failure: after the exception, the next step must redo the analysis before factorizing
// (a factorization after a failed Pardiso analysis crashes), and then succeed.
TEST(CH_timestepper, HHT_recover_after_setup_failure) {
    ChSystemSMC sys;
    auto solver = CreatePendulum(sys, ChTimestepper::Type::HHT);

    ASSERT_NO_THROW(sys.DoStepDynamics(1e-3));
    ASSERT_NO_THROW(sys.DoStepDynamics(1e-3));
    double time = sys.GetChTime();

    solver->fail_next = true;
    EXPECT_THROW(sys.DoStepDynamics(1e-3), std::runtime_error);
    EXPECT_EQ(solver->num_failed_setups, 1);
    EXPECT_EQ(sys.GetChTime(), time);  // HHT scatters its predictor at T + h before the solve; T must be restored

    EXPECT_NO_THROW(sys.DoStepDynamics(1e-3));
    EXPECT_EQ(solver->num_unfactored_solves, 0);
    EXPECT_EQ(solver->num_unanalyzed_setups, 0);
}
