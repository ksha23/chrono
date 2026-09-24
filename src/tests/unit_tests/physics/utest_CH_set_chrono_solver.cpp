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
// Check the solver selected by the demo utility SetChronoSolver when a requested
// direct sparse solver module (PardisoMKL or MUMPS) is not enabled.
// The fallback must be SPARSE_LU (not the much slower SPARSE_QR).
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;

static ChSolver::Type Selected(ChSystem& sys, ChSolver::Type requested) {
    bool ok = SetChronoSolver(sys, requested, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    EXPECT_TRUE(ok);
    return sys.GetSolver()->GetType();
}

TEST(SetChronoSolver, fallback_pardiso_mkl) {
    ChSystemSMC sys;
#ifdef CHRONO_PARDISO_MKL
    EXPECT_EQ(Selected(sys, ChSolver::Type::PARDISO_MKL), ChSolver::Type::PARDISO_MKL);
#else
    EXPECT_EQ(Selected(sys, ChSolver::Type::PARDISO_MKL), ChSolver::Type::SPARSE_LU);
#endif
}

TEST(SetChronoSolver, fallback_mumps) {
    ChSystemSMC sys;
#ifdef CHRONO_MUMPS
    EXPECT_EQ(Selected(sys, ChSolver::Type::MUMPS), ChSolver::Type::MUMPS);
#else
    EXPECT_EQ(Selected(sys, ChSolver::Type::MUMPS), ChSolver::Type::SPARSE_LU);
#endif
}

TEST(SetChronoSolver, explicit_direct_solvers) {
    // Explicit requests for the Eigen direct solvers are honored
    ChSystemNSC sys;
    EXPECT_EQ(Selected(sys, ChSolver::Type::SPARSE_QR), ChSolver::Type::SPARSE_QR);
    EXPECT_EQ(Selected(sys, ChSolver::Type::SPARSE_LU), ChSolver::Type::SPARSE_LU);
}
