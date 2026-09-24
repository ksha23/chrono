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
// Tests for the reuse of the symbolic analysis in direct sparse linear solvers:
// - implicit integrators only request an analysis when the system was modified
// - a change in the matrix sparsity pattern always triggers a new analysis
// - results are identical to those obtained with an analysis at every setup
//
// =============================================================================

#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

using namespace chrono;

// Sparse LU direct solver (same engine as ChSolverSparseLU) that counts analysis and factorization calls.
// If 'always_analyze' is true, every factorization is preceded by an analysis (reference behavior).
class CountingSparseLU : public ChDirectSolverLS {
  public:
    CountingSparseLU(bool always_analyze = false) : m_always_analyze(always_analyze) {}

    unsigned int num_analyze = 0;
    unsigned int num_factorize = 0;

  protected:
    virtual bool FactorizeMatrix(bool analyze) override {
        if (analyze || m_always_analyze) {
            m_engine.analyzePattern(m_mat);
            num_analyze++;
        }
        m_engine.factorize(m_mat);
        num_factorize++;
        return m_engine.info() == Eigen::Success;
    }

    virtual bool SolveSystem() override {
        m_sol = m_engine.solve(m_rhs);
        return m_engine.info() == Eigen::Success;
    }

    virtual void PrintErrorMessage() override {}

  private:
    bool m_always_analyze;
    Eigen::SparseLU<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;
};

// Chain of pendulum links connected with revolute joints (no contacts, so the system is never marked as modified).
static std::vector<std::shared_ptr<ChBody>> BuildChain(ChSystem& sys, int n) {
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    std::vector<std::shared_ptr<ChBody>> bodies;
    auto prev = ground;
    for (int i = 0; i < n; i++) {
        auto body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 0.1, 0.1, 1000, false, false);
        body->SetPos(ChVector3d(i + 0.5, 0, 0));
        sys.AddBody(body);
        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(prev, body, ChFramed(ChVector3d(i, 0, 0), QUNIT));
        sys.AddLink(rev);
        bodies.push_back(body);
        prev = body;
    }
    return bodies;
}

static ChVectorDynamic<> GetState(ChSystem& sys) {
    ChState x(sys.GetNumCoordsPosLevel(), &sys);
    ChStateDelta v(sys.GetNumCoordsVelLevel(), &sys);
    double t;
    sys.StateGather(x, v, t);
    ChVectorDynamic<> s(x.size() + v.size());
    s << x, v;
    return s;
}

struct RunResult {
    unsigned int num_analyze_first;  // analysis calls during the first step
    unsigned int num_analyze;
    unsigned int num_factorize;
    ChVectorDynamic<> state;
};

// Simulate the chain for 'num_steps' steps.
// If 'add_load_step' >= 0, a stiff bushing load coupling the first and last links (which changes the sparsity pattern
// without changing the problem size) is added before that step.
// If 'force_update' is true, the system is explicitly marked as modified before each step.
static RunResult RunChain(ChTimestepper::Type type, bool lock, bool always_analyze, int num_steps, int add_load_step = -1, bool force_update = false) {
    ChSystemSMC sys;
    auto bodies = BuildChain(sys, 8);

    auto loads = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(loads);

    auto solver = chrono_types::make_shared<CountingSparseLU>(always_analyze);
    solver->LockSparsityPattern(lock);
    sys.SetSolver(solver);
    sys.SetTimestepperType(type);
    if (auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(50);
        hht->SetAbsTolerances(1e-4, 1e2);
        hht->SetStepControl(false);
    }

    unsigned int num_analyze_first = 0;
    for (int i = 0; i < num_steps; i++) {
        if (i == add_load_step) {
            auto bushing =
                chrono_types::make_shared<ChLoadBodyBodyBushingSpherical>(bodies.front(), bodies.back(), ChFramed(bodies.back()->GetPos()), ChVector3d(1e3), ChVector3d(1e1));
            loads->Add(bushing);
        }
        if (force_update)
            sys.ForceUpdate();
        sys.DoStepDynamics(1e-3);
        if (i == 0)
            num_analyze_first = solver->num_analyze;
    }

    return {num_analyze_first, solver->num_analyze, solver->num_factorize, GetState(sys)};
}

class DirectSolverAnalyze : public ::testing::TestWithParam<ChTimestepper::Type> {};

// With an unchanged system, the matrix structure is analyzed only during the first step (at each of its setups, as the
// system is initially marked as modified), regardless of whether the sparsity pattern is locked or not. Results must be bitwise identical to an analysis at every setup.
TEST_P(DirectSolverAnalyze, analyze_once) {
    for (bool lock : {true, false}) {
        auto res = RunChain(GetParam(), lock, false, 10);
        auto ref = RunChain(GetParam(), lock, true, 10);
        EXPECT_GE(res.num_analyze_first, 1u) << "lock=" << lock;
        EXPECT_EQ(res.num_analyze, res.num_analyze_first) << "lock=" << lock;
        EXPECT_GE(res.num_factorize, 10u) << "lock=" << lock;
        EXPECT_EQ(res.num_factorize, ref.num_factorize) << "lock=" << lock;
        ASSERT_EQ(res.state.size(), ref.state.size());
        for (int i = 0; i < res.state.size(); i++)
            ASSERT_EQ(res.state[i], ref.state[i]) << "lock=" << lock << " i=" << i;
    }
}

// A system marked as modified at every step is re-analyzed at every step.
TEST_P(DirectSolverAnalyze, modified_system) {
    auto res = RunChain(GetParam(), false, false, 10, -1, true);
    EXPECT_GE(res.num_analyze, 10u);
}

// A change of the sparsity pattern (without a change in problem size and without marking the system as modified)
// must trigger a new analysis. Results must be identical to an analysis at every setup.
TEST_P(DirectSolverAnalyze, pattern_change) {
    for (bool lock : {true, false}) {
        auto res = RunChain(GetParam(), lock, false, 10, 5);
        auto ref = RunChain(GetParam(), lock, true, 10, 5);
        EXPECT_EQ(res.num_analyze, res.num_analyze_first + 1) << "lock=" << lock;
        ASSERT_EQ(res.state.size(), ref.state.size());
        for (int i = 0; i < res.state.size(); i++)
            ASSERT_EQ(res.state[i], ref.state[i]) << "lock=" << lock << " i=" << i;
    }
}

INSTANTIATE_TEST_SUITE_P(Timesteppers,
                         DirectSolverAnalyze,
                         ::testing::Values(ChTimestepper::Type::EULER_IMPLICIT,
                                           ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED,
                                           ChTimestepper::Type::EULER_IMPLICIT_PROJECTED,
                                           ChTimestepper::Type::TRAPEZOIDAL,
                                           ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED,
                                           ChTimestepper::Type::NEWMARK,
                                           ChTimestepper::Type::HHT));

// SetupCurrent(false) reuses the analysis if only matrix values changed, and re-analyzes if the structure changed.
TEST(DirectSolverAnalyzeCurrent, setup_current) {
    const int n = 20;
    CountingSparseLU solver;
    auto& A = solver.A();
    A.resize(n, n);
    for (int i = 0; i < n; i++) {
        A.coeffRef(i, i) = 4.0;
        if (i > 0)
            A.coeffRef(i, i - 1) = -1.0;
        if (i < n - 1)
            A.coeffRef(i, i + 1) = -1.0;
    }
    solver.b().setOnes(n);

    auto check_solution = [&]() {
        solver.SolveCurrent();
        double res = (solver.A() * solver.x() - solver.b()).lpNorm<Eigen::Infinity>();
        EXPECT_LT(res, 1e-12);
    };

    ASSERT_TRUE(solver.SetupCurrent());
    EXPECT_EQ(solver.num_analyze, 1u);
    check_solution();

    // Change diagonal values only: analysis reused
    for (int i = 0; i < n; i++)
        A.coeffRef(i, i) += 1.0;
    ASSERT_TRUE(solver.SetupCurrent(false));
    EXPECT_EQ(solver.num_analyze, 1u);
    EXPECT_EQ(solver.num_factorize, 2u);
    check_solution();

    // Add new nonzeros: analysis forced even if not requested
    A.coeffRef(0, n - 1) = 0.5;
    A.coeffRef(n - 1, 0) = 0.5;
    ASSERT_TRUE(solver.SetupCurrent(false));
    EXPECT_EQ(solver.num_analyze, 2u);
    check_solution();

    // Default call always analyzes
    ASSERT_TRUE(solver.SetupCurrent());
    EXPECT_EQ(solver.num_analyze, 3u);
}
