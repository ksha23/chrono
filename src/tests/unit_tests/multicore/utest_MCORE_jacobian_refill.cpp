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
// Unit test for refilling the NSC constraint Jacobian (D_T) with fewer nonzeros
// than the previous fill.
// D_T is reused across steps. When the new contact set is smaller than the
// previous one, the storage kept from the previous fill must not push the
// sparse inserts onto Eigen's slow path (an O(rows) re-layout per insert).
// The test fills D_T once with a larger contact set, then again with a smaller
// one, and checks that the second fill produces exactly the matrix obtained
// from an empty D_T, in time comparable to that fresh fill.
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include "gtest/gtest.h"

using namespace chrono;

// Run ComputeD and return the elapsed wall time in seconds.
static double TimeComputeD(ChIterativeSolverMulticoreNSC& solver) {
    auto t0 = std::chrono::steady_clock::now();
    solver.ComputeD();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

TEST(ChronoMulticore, jacobian_refill) {
    ChSystemMulticoreNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys.SetNumThreads(1);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = 20;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 0;
    sys.GetSettings()->collision.collision_envelope = 0.005;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
    sys.ChangeSolverType(SolverType::APGD);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    // Lattice of spheres with gaps inside the collision envelope, so that every neighbor pair is a contact
    const int n = 10;
    const double r = 0.1;
    const double spacing = 2.02 * r;
    const double half = n * spacing / 2;
    utils::CreateBoxContainer(&sys, mat, ChVector3d(2 * half, 2 * half, 2 * n * spacing), 0.1);

    std::vector<std::shared_ptr<ChBody>> top;
    for (int k = 0; k < n; k++) {
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) {
                auto body = chrono_types::make_shared<ChBody>();
                body->SetMass(1);
                body->SetInertiaXX(0.4 * r * r * ChVector3d(1, 1, 1));
                body->SetPos(ChVector3d(-half + r + i * spacing, -half + r + j * spacing, r + 0.001 + k * spacing));
                body->EnableCollision(true);
                utils::AddSphereGeometry(body.get(), mat, r);
                sys.AddBody(body);
                if (k == n - 1 && i < 4 && j < 4)
                    top.push_back(body);
            }
        }
    }

    auto solver = std::dynamic_pointer_cast<ChIterativeSolverMulticoreNSC>(sys.GetSolver());
    ASSERT_TRUE(solver);
    auto& D_T = sys.data_manager->host_data.D_T;

    // First fill: full lattice.
    sys.DoStepDynamics(1e-3);
    unsigned int nc1 = sys.data_manager->cd_data->num_rigid_contacts;
    ASSERT_GT(nc1, 2000u);
    SparseMatrixType previous = D_T;  // keeps the allocation of the larger fill

    // Lift a few bodies out of the pile so that the next step has fewer contacts.
    for (auto& body : top) {
        body->SetPos(body->GetPos() + ChVector3d(0, 0, 10));
        body->SetPosDt(VNULL);
    }
    sys.DoStepDynamics(1e-3);
    unsigned int nc2 = sys.data_manager->cd_data->num_rigid_contacts;
    ASSERT_LT(nc2, nc1);
    ASSERT_GT(nc2, 2000u);

    // Reference: fill the smaller contact set into an empty matrix (best of 3).
    double t_fresh = 1e30;
    for (int rep = 0; rep < 3; rep++) {
        D_T = SparseMatrixType();
        t_fresh = std::min(t_fresh, TimeComputeD(*solver));
    }
    SparseMatrixType ref = D_T;
    ASSERT_EQ(ref.rows(), 3 * (Eigen::Index)nc2);

    // Refill the smaller contact set into the matrix left by the larger fill (best of 3).
    double t_refill = 1e30;
    for (int rep = 0; rep < 3; rep++) {
        D_T = previous;
        ASSERT_GT(D_T.data().allocatedSize(), ref.nonZeros());
        t_refill = std::min(t_refill, TimeComputeD(*solver));
    }

    // Same matrix, bit for bit.
    ASSERT_TRUE(D_T.isCompressed());
    ASSERT_EQ(D_T.rows(), ref.rows());
    ASSERT_EQ(D_T.cols(), ref.cols());
    ASSERT_EQ(D_T.nonZeros(), ref.nonZeros());
    const auto nnz = (size_t)ref.nonZeros();
    EXPECT_EQ(0, std::memcmp(D_T.outerIndexPtr(), ref.outerIndexPtr(), (ref.outerSize() + 1) * sizeof(int)));
    EXPECT_EQ(0, std::memcmp(D_T.innerIndexPtr(), ref.innerIndexPtr(), nnz * sizeof(int)));
    EXPECT_EQ(0, std::memcmp(D_T.valuePtr(), ref.valuePtr(), nnz * sizeof(real)));

    // Comparable cost. With the stale allocation, every insert beyond the first few rows re-lays out the whole matrix
    // and the refill is two orders of magnitude slower than the fresh fill.
    std::cout << "contacts " << nc1 << " -> " << nc2 << ", fresh fill " << 1e3 * t_fresh << " ms, refill " << 1e3 * t_refill << " ms" << std::endl;
    EXPECT_LT(t_refill, 5 * t_fresh + 0.02);
}
