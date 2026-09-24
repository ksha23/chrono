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
// Chrono::Multicore unit test for the row-parallel sparse matrix-vector product
// used by the Schur product (ChSchurProduct::SpMV).
// Checks the result against Eigen's serial product and checks that it is bitwise
// identical for any number of OpenMP threads.
// =============================================================================

#include <cmath>
#include <random>
#include <vector>

#include "chrono/utils/ChOpenMP.h"
#include "chrono_multicore/solver/ChSolverMulticore.h"

#include "gtest/gtest.h"

using namespace chrono;

// Random row-major matrix with about nnz_per_row entries per row and some empty rows.
static SparseMatrixType RandomMatrix(int rows, int cols, int nnz_per_row, bool compressed, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> col(0, cols - 1);
    std::uniform_int_distribution<int> count(0, 2 * nnz_per_row);
    std::uniform_real_distribution<double> val(-1.0, 1.0);

    SparseMatrixType A(rows, cols);
    A.reserve(Eigen::VectorXi::Constant(rows, 2 * nnz_per_row));
    for (int i = 0; i < rows; i++) {
        if (i % 17 == 0)
            continue;
        int n = count(rng);
        for (int k = 0; k < n; k++)
            A.coeffRef(i, col(rng)) += val(rng);
    }
    if (compressed)
        A.makeCompressed();
    return A;
}

static VectorType RandomVector(int n, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> val(-1.0, 1.0);
    VectorType v(n);
    for (int i = 0; i < n; i++)
        v(i) = val(rng);
    return v;
}

// Compare against the serial Eigen product, with a bound on the rounding error of a row reduction.
static void CheckAgainstEigen(const SparseMatrixType& A, const VectorType& x, const VectorType& y0, const VectorType& y) {
    VectorType ref = y0;
    ref.noalias() += A * x;
    SparseMatrixType absA = A.cwiseAbs();
    VectorType bound = y0.cwiseAbs() + absA * x.cwiseAbs();
    for (Eigen::Index i = 0; i < y.size(); i++)
        ASSERT_NEAR(y(i), ref(i), 1e-14 * (1 + bound(i))) << "row " << i;
}

class SchurSpMV : public ::testing::TestWithParam<bool> {};

TEST_P(SchurSpMV, product) {
    bool compressed = GetParam();
    int saved_threads = ChOMP::GetMaxThreads();

    // Large enough to take the parallel path, and a small one that stays serial
    for (int rows : {60000, 500}) {
        int cols = rows / 5;
        SparseMatrixType A = RandomMatrix(rows, cols, 12, compressed, 42u + rows);
        ASSERT_EQ(A.isCompressed(), compressed);
        VectorType x = RandomVector(cols, 7u);
        VectorType y0 = RandomVector(rows, 11u);

        // y = A * x (the output is overwritten, not accumulated)
        ChOMP::SetNumThreads(1);
        VectorType y_serial = VectorType::Constant(rows, 123.0);
        ChSchurProduct::SpMV(A, x, y_serial);
        CheckAgainstEigen(A, x, VectorType::Zero(rows), y_serial);

        // y += A * x
        VectorType z_serial = y0;
        ChSchurProduct::SpMV(A, x, z_serial, true);
        CheckAgainstEigen(A, x, y0, z_serial);

        // Bitwise identical results for any number of threads
        for (int nthreads : {2, 3, 8}) {
            ChOMP::SetNumThreads(nthreads);
            VectorType y = VectorType::Constant(rows, -5.0);
            ChSchurProduct::SpMV(A, x, y);
            VectorType z = y0;
            ChSchurProduct::SpMV(A, x, z, true);
            for (int i = 0; i < rows; i++) {
                ASSERT_EQ(y(i), y_serial(i)) << "threads " << nthreads << " row " << i;
                ASSERT_EQ(z(i), z_serial(i)) << "threads " << nthreads << " row " << i;
            }
        }
    }

    ChOMP::SetNumThreads(saved_threads);
}

TEST(SchurSpMVSegments, accumulate) {
    // Product on vector segments, as used by the Schur product for the normal/tangential blocks
    int rows = 30000, cols = 9000;
    SparseMatrixType A = RandomMatrix(rows, cols, 10, true, 3u);
    VectorType x_full = RandomVector(cols + 100, 5u);
    VectorType out_full = RandomVector(rows + 200, 9u);
    VectorType out_ref = out_full;

    ConstSubVectorType x = static_cast<const VectorType&>(x_full).segment(50, cols);
    SubVectorType out = out_full.segment(100, rows);
    ChSchurProduct::SpMV(A, x, out, true);

    VectorType x_copy = x;
    VectorType y0 = out_ref.segment(100, rows);
    CheckAgainstEigen(A, x_copy, y0, out_full.segment(100, rows));

    // Entries outside the segment are untouched
    for (int i = 0; i < 100; i++)
        ASSERT_EQ(out_full(i), out_ref(i));
    for (int i = 100 + rows; i < rows + 200; i++)
        ASSERT_EQ(out_full(i), out_ref(i));
}

INSTANTIATE_TEST_SUITE_P(MCORE, SchurSpMV, ::testing::Values(true, false));
