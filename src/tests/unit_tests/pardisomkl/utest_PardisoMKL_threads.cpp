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
// Authors: Kyle Sha
// =============================================================================
//
// Test multithreaded MKL Pardiso.
// - The process must not load two different OpenMP runtimes (e.g. the GNU runtime
//   used by Chrono and the Intel/LLVM runtime used by MKL's intel_thread layer).
// - A GCC build with OpenMP must use MKL's gnu_thread layer.
// - A sparse system factorized with 1, 2, 4 and 8 MKL threads must be solved
//   successfully, with results matching the sequential solution.
//
// =============================================================================

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/core/ChMatrix.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#if defined(__linux__)
    #include <link.h>
#endif

#include "gtest/gtest.h"

using namespace chrono;

#if defined(__linux__)
// Collect the resolved paths of all loaded shared libraries (symbolic links are resolved, since some
// distributions provide libgomp.so.1 or libiomp5.so as links to the LLVM OpenMP runtime).
static int CollectLibrary(struct dl_phdr_info* info, size_t size, void* data) {
    auto libs = static_cast<std::vector<std::string>*>(data);
    if (info->dlpi_name && info->dlpi_name[0] != '\0') {
        char* path = realpath(info->dlpi_name, nullptr);
        std::string name = path ? path : info->dlpi_name;
        free(path);
        libs->push_back(name.substr(name.find_last_of('/') + 1));
    }
    return 0;
}

static std::vector<std::string> LoadedLibraries() {
    // Make sure the Pardiso module (and hence MKL) is actually used by this process
    ChSolverPardisoMKL solver;
    (void)solver;

    std::vector<std::string> libs;
    dl_iterate_phdr(CollectLibrary, &libs);
    return libs;
}

static bool IsLoaded(const std::vector<std::string>& libs, const std::string& prefix) {
    for (const auto& lib : libs) {
        if (lib.rfind(prefix, 0) == 0)
            return true;
    }
    return false;
}
#endif

// Check that at most one OpenMP runtime is loaded in the process.
TEST(PardisoMKL, single_openmp_runtime) {
#if defined(__linux__)
    auto libs = LoadedLibraries();
    bool gnu = IsLoaded(libs, "libgomp.so");
    bool llvm = IsLoaded(libs, "libiomp5.so") || IsLoaded(libs, "libomp.so");
    std::cout << "GNU OpenMP runtime loaded: " << gnu << "   Intel/LLVM OpenMP runtime loaded: " << llvm << std::endl;
    EXPECT_FALSE(gnu && llvm) << "Both libgomp and libiomp5/libomp are loaded; MKL_THREADING does not match the "
                                 "OpenMP runtime used by Chrono";
#else
    GTEST_SKIP() << "Loaded library check only implemented on Linux";
#endif
}

// Check that a GCC build with OpenMP does not use the MKL threading layer for the Intel OpenMP runtime.
// This is the check that detects a mismatched configuration regardless of the library load order. The
// runtime collision itself only shows up when libgomp is loaded before libiomp5/libomp (as in the vehicle
// demos, or with LD_PRELOAD); in that case the other two tests fail as well. In the load order of this test
// executable alone, the collision may not occur (or libgomp may be an alias of the LLVM runtime).
// CMake also selects gnu_thread for Clang builds that link libgomp. Those are not checked here, since a
// Clang build usually links the LLVM runtime (libomp), for which intel_thread is the correct layer, and
// the compiler macros do not tell which runtime was linked.
TEST(PardisoMKL, threading_layer) {
#if defined(__linux__) && defined(_OPENMP) && defined(__GNUC__) && !defined(__clang__) && !defined(__INTEL_COMPILER)
    auto libs = LoadedLibraries();
    bool intel_thread = IsLoaded(libs, "libmkl_intel_thread.so");
    bool gnu_thread = IsLoaded(libs, "libmkl_gnu_thread.so");
    std::cout << "MKL intel_thread layer loaded: " << intel_thread << "   MKL gnu_thread layer loaded: " << gnu_thread << std::endl;
    EXPECT_FALSE(intel_thread) << "Chrono uses the GNU OpenMP runtime, but MKL uses the intel_thread layer; "
                                  "configure with MKL_THREADING=gnu_thread";
#else
    GTEST_SKIP() << "Only relevant for GCC builds with OpenMP on Linux";
#endif
}

// Assemble a nonsymmetric sparse matrix (3D 7-point Laplacian plus a convection term).
static void BuildMatrix(int n, ChSparseMatrix& A, ChVectorDynamic<>& b) {
    int N = n * n * n;
    auto idx = [n](int i, int j, int k) { return (i * n + j) * n + k; };
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(7 * N);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                int r = idx(i, j, k);
                triplets.emplace_back(r, r, 6.5);
                if (i > 0)
                    triplets.emplace_back(r, idx(i - 1, j, k), -1.2);
                if (i < n - 1)
                    triplets.emplace_back(r, idx(i + 1, j, k), -0.8);
                if (j > 0)
                    triplets.emplace_back(r, idx(i, j - 1, k), -1.0);
                if (j < n - 1)
                    triplets.emplace_back(r, idx(i, j + 1, k), -1.0);
                if (k > 0)
                    triplets.emplace_back(r, idx(i, j, k - 1), -1.0);
                if (k < n - 1)
                    triplets.emplace_back(r, idx(i, j, k + 1), -1.0);
            }
        }
    }
    A.resize(N, N);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();

    b.resize(N);
    for (int r = 0; r < N; r++)
        b(r) = std::sin(0.01 * r) + 1.0;
}

// Factorize and solve the same system with an increasing number of MKL threads.
TEST(PardisoMKL, multithreaded_solve) {
    ChSparseMatrix A;
    ChVectorDynamic<> b;
    BuildMatrix(30, A, b);

    // Set the number of threads of Chrono's OpenMP runtime, as a simulation would
    ChOMP::SetNumThreads(2);

    ChVectorDynamic<> x_ref;
    for (int num_threads : {1, 2, 4, 8}) {
        ChSolverPardisoMKL solver(num_threads);
        auto& engine = solver.GetMklEngine();

        engine.analyzePattern(A);
        ASSERT_EQ(engine.info(), Eigen::Success) << "ANALYZE failed with " << num_threads << " MKL threads";
        engine.factorize(A);
        ASSERT_EQ(engine.info(), Eigen::Success) << "FACTORIZE failed with " << num_threads << " MKL threads";
        ChVectorDynamic<> x = engine.solve(b);
        ASSERT_EQ(engine.info(), Eigen::Success) << "SOLVE failed with " << num_threads << " MKL threads";

        double res = (A * x - b).norm() / b.norm();
        std::cout << "MKL threads: " << num_threads << "   relative residual: " << res << std::endl;
        EXPECT_LT(res, 1e-12);

        if (num_threads == 1) {
            x_ref = x;
        } else {
            double diff = (x - x_ref).norm() / x_ref.norm();
            EXPECT_LT(diff, 1e-12) << "Solution with " << num_threads << " MKL threads differs from sequential";
        }
    }
}
