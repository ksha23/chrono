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
// Tests for the APGD solver (ChSolverAPGD).
// - a small mixed linear complementarity problem (2 bilateral, 3 unilateral constraints) with a known solution
// - a box resting on the ground (NSC frictional contact), where the contact force must balance the weight
//
// =============================================================================

#include <cmath>
#include <memory>
#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariablesGeneric.h"

#include "gtest/gtest.h"

using namespace chrono;

// Mixed LCP on three 3-DOF variable blocks with diagonal masses 1, 10, 100.
// Constraints 0 and 1 are bilateral, constraints 2, 3 and 4 are unilateral.
// At the solution, unilateral constraints 2 and 4 are inactive (zero multiplier) and constraint 3 is active.
class MixedLCP {
  public:
    MixedLCP() {
        const double mass[3] = {1, 10, 100};
        const double force[3][3] = {{1, -2, 0.5}, {0, 3, -1}, {-2, 0, 4}};

        struct ConstraintData {
            int a, b;
            double Cq_a[3], Cq_b[3];
            double rhs;
            bool unilateral;
        };
        const ConstraintData cdata[5] = {{0, 1, {1, 2, -1}, {1, -2, 0}, -0.5, false},  //
                                         {1, 2, {0, 1, 0.5}, {0, -1, 1}, 0.2, false},  //
                                         {0, 2, {0, 1, 0}, {1, 0, 0}, 0.3, true},      //
                                         {0, 1, {-1, 0, 1}, {0, 0, 1}, -0.4, true},    //
                                         {1, 2, {1, 1, 0}, {0, 1, -1}, 0.1, true}};

        descriptor.BeginInsertion();
        for (int i = 0; i < 3; i++) {
            variables.push_back(std::make_unique<ChVariablesGeneric>(3));
            auto& v = *variables.back();
            v.GetMass() = mass[i] * ChMatrixDynamic<>::Identity(3, 3);
            v.GetInvMass() = (1 / mass[i]) * ChMatrixDynamic<>::Identity(3, 3);
            for (int k = 0; k < 3; k++)
                v.Force()(k) = force[i][k];
            descriptor.InsertVariables(&v);
        }
        for (const auto& cd : cdata) {
            constraints.push_back(std::make_unique<ChConstraintTwoGeneric>(variables[cd.a].get(), variables[cd.b].get()));
            auto& c = *constraints.back();
            for (int k = 0; k < 3; k++) {
                c.Get_Cq_a()(k) = cd.Cq_a[k];
                c.Get_Cq_b()(k) = cd.Cq_b[k];
            }
            c.SetRightHandSide(cd.rhs);
            c.SetMode(cd.unilateral ? ChConstraint::Mode::UNILATERAL : ChConstraint::Mode::LOCK);
            descriptor.InsertConstraint(&c);
        }
        descriptor.EndInsertion();
    }

    ChSystemDescriptor descriptor;
    std::vector<std::unique_ptr<ChVariablesGeneric>> variables;
    std::vector<std::unique_ptr<ChConstraintTwoGeneric>> constraints;
};

TEST(ChSolverAPGD, mixed_lcp) {
    // Reference multipliers: exact solution of the MLCP (active set {0, 1, 3}), also reached by PSOR to round-off
    const double l_ref[5] = {1.1294996266, -2.3760268857, 0.0, 1.6084764750, 0.0};

    for (int iters : {100, 500}) {
        MixedLCP lcp;

        ChSolverAPGD solver;
        solver.SetMaxIterations(iters);
        solver.SetTolerance(0);
        solver.EnableWarmStart(false);
        solver.Solve(lcp.descriptor);

        ASSERT_TRUE(std::isfinite(solver.GetError()));
        EXPECT_LT(solver.GetError(), 1e-6) << "iterations: " << iters;

        for (int i = 0; i < 5; i++) {
            double l = lcp.constraints[i]->GetLagrangeMultiplier();
            ASSERT_TRUE(std::isfinite(l));
            EXPECT_NEAR(l, l_ref[i], 1e-6) << "constraint " << i << ", iterations: " << iters;
        }

        // Complementarity conditions and constraint residuals of the solution
        double max_res, max_lcp_err;
        lcp.descriptor.ComputeFeasibilityViolation(max_res, max_lcp_err);
        EXPECT_LT(max_res, 1e-6);
        EXPECT_LT(max_lcp_err, 1e-6);
    }
}

TEST(ChSolverAPGD, box_on_ground) {
    const double mass = 10;
    const double gacc = 9.81;
    const double step = 1e-3;

    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gacc));
    sys.SetSolverType(ChSolver::Type::APGD);
    sys.GetSolver()->AsIterative()->SetMaxIterations(100);
    sys.GetSolver()->AsIterative()->SetTolerance(0);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.5f);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(4, 4, 0.2, 1000, true, true, mat);
    ground->SetPos(ChVector3d(0, 0, -0.1));
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.3, 0.2, mass / (0.4 * 0.3 * 0.2), true, true, mat);
    box->SetPos(ChVector3d(0, 0, 0.1));
    sys.AddBody(box);

    while (sys.GetChTime() < 0.5) {
        sys.DoStepDynamics(step);
        ASSERT_TRUE(std::isfinite(box->GetPos().z()));
    }

    // The box must rest on the ground and the contact force must balance its weight
    ChVector3d fc = sys.GetContactContainer()->GetContactableForce(box.get());
    EXPECT_NEAR(fc.z(), mass * gacc, 1e-2 * mass * gacc);
    EXPECT_NEAR(fc.x(), 0, 1e-2 * mass * gacc);
    EXPECT_NEAR(fc.y(), 0, 1e-2 * mass * gacc);
    EXPECT_LT(box->GetPosDt().Length(), 1e-3);
    EXPECT_NEAR(box->GetPos().z(), 0.1, 5e-3);
}
