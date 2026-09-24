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
// Check the default linear solver used by ChModalAssembly for K_IIc^{-1} and
// verify that it gives the same reduced model and dynamics as SparseQR.
// The model is a cantilever beam with an internal body attached through an
// internal constraint, so that K_IIc is a saddle-point matrix.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include <Eigen/Eigenvalues>

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

struct ReducedResult {
    ChVectorDynamic<> eig;    // eigenvalues of the reduced pencil (modal_K, modal_M), sorted
    ChMatrixDynamic<> Psi_S;  // static modes (boundary columns of Psi); independent of eigenvector signs
    ChVectorDynamic<> tip;    // tip node position history
};

static double RelDiff(const ChMatrixDynamic<>& a, const ChMatrixDynamic<>& b) {
    return (a - b).lpNorm<Eigen::Infinity>() / std::max(1.0, b.lpNorm<Eigen::Infinity>());
}

static ReducedResult RunModel(ChModalAssembly::ReductionType type, std::shared_ptr<ChDirectSolverLS> modal_solver) {
    const int n_elements = 12;
    const double L = 6;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(VNULL);
    sys.SetSolver(chrono_types::make_shared<ChSolverSparseLU>());

    auto assembly = chrono_types::make_shared<ChModalAssembly>();
    assembly->SetReductionType(type);
    assembly->SetUseStaticCorrection(true);
    assembly->SetInternalNodesUpdate(true);
    if (modal_solver)
        assembly->SetModalSolver(modal_solver);
    sys.Add(assembly);

    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    assembly->AddInternal(mesh_internal);
    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    assembly->Add(mesh_boundary);

    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    section->SetDensity(1000);
    section->SetYoungModulus(100e6);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(0.01);
    section->SetAsRectangularSection(0.05, 0.3);

    auto node_A = chrono_types::make_shared<ChNodeFEAxyzrot>();
    mesh_boundary->AddNode(node_A);
    auto node_B = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(L, 0, 0)));
    mesh_boundary->AddNode(node_B);

    ChBuilderBeamEuler builder;
    builder.BuildBeam(mesh_internal, section, n_elements, node_A, node_B, ChVector3d(0, 1, 0));

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(-0.5, 0, 0));
    assembly->Add(ground);
    auto root = chrono_types::make_shared<ChLinkMateGeneric>();
    root->Initialize(node_A, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    assembly->Add(root);

    // Internal body attached through an internal constraint (K_IIc becomes a saddle-point matrix)
    auto body_mid = chrono_types::make_shared<ChBodyEasyBox>(0.9, 1.4, 1.2, 100);
    body_mid->SetPos(ChVector3d(L / 2, 0, 0));
    assembly->AddInternal(body_mid);
    auto mid_constr = chrono_types::make_shared<ChLinkMateGeneric>();
    mid_constr->Initialize(builder.GetLastBeamNodes()[n_elements / 2], body_mid, ChFrame<>(ChVector3d(L / 2, 0, 0), QUNIT));
    assembly->AddInternal(mid_constr);

    auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
    ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_eig(8, 1e-5, true, false, eigen_solver);
    assembly->DoModalReduction(modal_eig, ChModalDampingRayleigh(0.001, 0.01));
    sys.Setup();
    sys.Update(UpdateFlags::UPDATE_ALL);

    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    hht->SetStepControl(false);
    hht->SetAlpha(-0.2);

    // Eigenvalues of the reduced model are invariant to the sign/normalization of the retained eigenvectors,
    // so they can be compared directly between runs
    ReducedResult res;
    const ChMatrixDynamic<>& Mr = assembly->GetModalMassMatrix();
    const ChMatrixDynamic<>& Kr = assembly->GetModalStiffnessMatrix();
    Eigen::GeneralizedEigenSolver<ChMatrixDynamic<>> ges(Kr, Mr, false);
    res.eig = ges.eigenvalues().real();
    std::sort(res.eig.data(), res.eig.data() + res.eig.size());
    int nB = assembly->GetNumCoordinatesVelBoundary();
    res.Psi_S = assembly->GetModalReductionMatrix().leftCols(nB);

    node_B->SetForce(ChVector3d(0, -3, 2));
    const int num_steps = 50;
    res.tip.resize(3 * num_steps);
    for (int i = 0; i < num_steps; i++) {
        sys.DoStepDynamics(0.005);
        res.tip.segment(3 * i, 3) = node_B->GetPos().eigen();
    }

    return res;
}

TEST(ChModalAssembly, default_modal_solver) {
    ChModalAssembly assembly;
    ASSERT_TRUE(assembly.GetModalSolver());
    EXPECT_EQ(assembly.GetModalSolver()->GetType(), ChSolver::Type::SPARSE_LU);
}

class ModalSolverLUvsQR : public ::testing::TestWithParam<ChModalAssembly::ReductionType> {};

TEST_P(ModalSolverLUvsQR, reduced_model_and_dynamics) {
    auto type = GetParam();
    auto lu = RunModel(type, chrono_types::make_shared<ChSolverSparseLU>());
    auto qr = RunModel(type, chrono_types::make_shared<ChSolverSparseQR>());

    const double tol = 1e-10;
    double d_eig = RelDiff(lu.eig, qr.eig);
    double d_Psi = RelDiff(lu.Psi_S, qr.Psi_S);
    double d_tip = RelDiff(lu.tip, qr.tip);
    std::cout << "LU vs QR relative differences: reduced eigenvalues " << d_eig << "  static modes " << d_Psi << "  tip history " << d_tip << std::endl;

    ASSERT_GT(lu.eig.size(), 0);
    EXPECT_TRUE(lu.eig.allFinite());
    EXPECT_TRUE(lu.tip.allFinite());
    EXPECT_GT((lu.tip.tail(3) - lu.tip.head(3)).norm(), 1e-4);  // the tip does move
    EXPECT_LT(d_eig, tol);
    EXPECT_LT(d_Psi, tol);
    EXPECT_LT(d_tip, tol);
}

INSTANTIATE_TEST_SUITE_P(ChModalAssembly, ModalSolverLUvsQR, ::testing::Values(ChModalAssembly::ReductionType::HERTING, ChModalAssembly::ReductionType::CRAIG_BAMPTON));
