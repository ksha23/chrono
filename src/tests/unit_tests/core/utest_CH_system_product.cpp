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
// Unit test for ChSystemDescriptor::SystemProduct, the matrix-free KKT product
// used by the iterative linear solvers (MINRES, GMRES, BiCGSTAB).
// The product is compared against the assembled system matrix (BuildSystemMatrix)
// multiplied by the same vector, with 1 and several threads, for a model with
// many KRM blocks and constraints on 1, 2, and 3 variable objects.
// With more than 1 thread, the test also checks that the parallel KRM path was taken.
//
// =============================================================================

#include <memory>
#include <random>
#include <vector>

#include "chrono/solver/ChConstraintTuple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintTwoTuples.h"
#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariablesGeneric.h"

#include "gtest/gtest.h"

using namespace chrono;

class SystemProductTest : public ::testing::Test {
  protected:
    // Build a chain of 3-DOF variables coupled by 6x6 KRM blocks, plus constraints of various types.
    // KRM blocks couple the variables in [krm_begin, krm_end) (all variables if krm_end < 0).
    void Build(int num_vars, bool with_tuples = true, int krm_begin = 0, int krm_end = -1) {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> U(-1.0, 1.0);

        for (int i = 0; i < num_vars; i++) {
            auto v = std::make_unique<ChVariablesGeneric>(3);
            v->GetMass().setZero();
            v->GetMass().diagonal() << 1 + U(rng) * 0.5, 2 + U(rng) * 0.5, 3 + U(rng) * 0.5;
            v->GetInvMass() = v->GetMass().inverse();
            vars.push_back(std::move(v));
        }

        // KRM blocks between consecutive variables (these share variables, as FEA elements share nodes)
        if (krm_end < 0)
            krm_end = num_vars;
        for (int i = krm_begin; i + 1 < krm_end; i++) {
            auto b = std::make_unique<ChKRMBlock>();
            b->SetVariables({vars[i].get(), vars[i + 1].get()});
            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    b->GetMatrix()(r, c) = U(rng);
            krm.push_back(std::move(b));
        }

        // Generic two-variable constraints, one with compliance
        for (int i = 0; i + 5 < num_vars; i += 50) {
            auto c = std::make_unique<ChConstraintTwoGeneric>(vars[i].get(), vars[i + 5].get());
            for (int k = 0; k < 3; k++) {
                c->Get_Cq_a()(k) = U(rng);
                c->Get_Cq_b()(k) = U(rng);
            }
            c->SetComplianceTerm(i == 0 ? 0.1 : 0.0);
            generic.push_back(std::move(c));
        }

        // Constraints between a 3-variable tuple (e.g., a mesh triangle) and a 1-variable tuple (e.g., a node).
        // The 3 variables are not consecutive, so their offsets differ.
        for (int i = 0; with_tuples && i + 30 < num_vars; i += 97) {
            auto ta = new ChConstraintTuple_3vars<3, 3, 3>(vars[i].get(), vars[i + 10].get(), vars[i + 20].get());
            auto tb = new ChConstraintTuple_1vars<3>(vars[i + 30].get());
            for (int k = 0; k < 3; k++) {
                ta->Cq1()(k) = U(rng);
                ta->Cq2()(k) = U(rng);
                ta->Cq3()(k) = U(rng);
                tb->Cq1()(k) = U(rng);
            }
            auto c = std::make_unique<ChConstraintTwoTuples>();
            c->SetTuples(ta, tb);
            tuples.push_back(std::move(c));
        }

        sd.BeginInsertion();
        for (auto& v : vars)
            sd.InsertVariables(v.get());
        for (auto& b : krm)
            sd.InsertKRMBlock(b.get());
        for (auto& c : generic)
            sd.InsertConstraint(c.get());
        for (auto& c : tuples)
            sd.InsertConstraint(c.get());
        sd.EndInsertion();
        sd.SetMassFactor(0.7);

        int n = sd.CountActiveVariables() + sd.CountActiveConstraints();
        x.resize(n);
        for (int i = 0; i < n; i++)
            x(i) = U(rng);

        ChSparseMatrix Z;
        sd.BuildSystemMatrix(&Z, nullptr);
        Zx = Z * x;
    }

    // Check the matrix-free products against the assembled product for several thread counts. The results must be identical
    // between repeated calls with a given number of threads. With more than 1 thread, the parallel KRM path sums the KRM
    // terms in a different order than the serial loop, so a result bitwise equal to the serial one means the parallel path
    // was not taken (only checked when this test is compiled with OpenMP).
    void CheckThreads() {
        ChVectorDynamic<> r_serial;

        for (int nthreads : {1, 2, 3, 4, 8}) {
            sd.SetNumThreads(nthreads);
            ASSERT_EQ(sd.GetNumThreads(), nthreads);

            ChVectorDynamic<> r1, r2;
            sd.SystemProduct(r1, x);
            sd.SystemProduct(r2, x);
            EXPECT_LT(RelativeError(r1), 1e-14) << "threads = " << nthreads;
            EXPECT_TRUE(r1 == r2) << "threads = " << nthreads;

            if (nthreads == 1)
                r_serial = r1;
#ifdef _OPENMP
            else
                EXPECT_FALSE(r1 == r_serial) << "parallel KRM path not taken, threads = " << nthreads;
#endif

            // Upper part only: H*v + Cq'*l
            int nq = sd.CountActiveVariables();
            int nc = sd.CountActiveConstraints();
            ChVectorDynamic<> ru;
            sd.SystemProductUpper(ru, x.head(nq), x.tail(nc), false);
            EXPECT_LT((ru - Zx.head(nq)).norm() / Zx.head(nq).norm(), 1e-14) << "threads = " << nthreads;
        }
    }

    // Relative difference between the matrix-free product and the assembled product
    double RelativeError(const ChVectorDynamic<>& r) const { return (r - Zx).norm() / Zx.norm(); }

    std::vector<std::unique_ptr<ChVariablesGeneric>> vars;
    std::vector<std::unique_ptr<ChKRMBlock>> krm;
    std::vector<std::unique_ptr<ChConstraintTwoGeneric>> generic;
    std::vector<std::unique_ptr<ChConstraintTwoTuples>> tuples;
    ChSystemDescriptor sd;
    ChVectorDynamic<> x;
    ChVectorDynamic<> Zx;
};

// Small system, single 3-variable tuple: the constraint row must use the offset of the third variable.
TEST_F(SystemProductTest, tuple_3vars) {
    Build(40);
    ASSERT_EQ(tuples.size(), 1);

    ChVectorDynamic<> r;
    sd.SystemProduct(r, x);
    ASSERT_EQ(r.size(), Zx.size());
    EXPECT_LT(RelativeError(r), 1e-14);
    EXPECT_LT((r - Zx).cwiseAbs().maxCoeff(), 1e-12 * Zx.cwiseAbs().maxCoeff());
}

// Large system (enough KRM blocks for the parallel path), with 3-variable tuples.
TEST_F(SystemProductTest, krm_threads) {
    Build(1000);
    CheckThreads();
}

// Same, without 3-variable tuples, so that only the parallel KRM product is tested.
TEST_F(SystemProductTest, krm_threads_no_tuples) {
    Build(1000, false);
    ASSERT_EQ(tuples.size(), 0);
    CheckThreads();
}

// KRM blocks on a subset of the variables only: rows outside the range touched by KRM blocks must be correct too.
TEST_F(SystemProductTest, krm_threads_partial_range) {
    Build(1500, true, 300, 1300);
    CheckThreads();
}
