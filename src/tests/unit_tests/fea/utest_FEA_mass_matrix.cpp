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
// Unit test for element mass matrices and the mesh mass-times-vector product.
// For each element type, ComputeMmatrixGlobal must return exactly the mass part of
// ComputeKRMmatricesGlobal(H, 0, 0, 1), in a rotated and deformed configuration,
// and ChMesh::IntLoadResidual_Mv must match the assembled element mass matrices.
//
// =============================================================================

#include <cmath>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChMaterialShellKirchhoff.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

using NodeList = std::vector<std::shared_ptr<ChNodeFEAxyz>>;

// Hexahedron corner and mid-edge nodes, same ordering as demo_FEA_basic
static NodeList HexaNodes(ChMesh& mesh, bool quadratic) {
    double s = 0.1;
    std::vector<ChVector3d> c = {{0, 0, 0}, {0, 0, s}, {s, 0, s}, {s, 0, 0}, {0, s, 0}, {0, s, s}, {s, s, s}, {s, s, 0}};
    if (quadratic) {
        int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {0, 3}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {1, 5}, {2, 6}, {3, 7}, {0, 4}};
        for (auto& ed : e)
            c.push_back((c[ed[0]] + c[ed[1]]) * 0.5);
    }
    NodeList nodes;
    for (auto& p : c) {
        nodes.push_back(chrono_types::make_shared<ChNodeFEAxyz>(p));
        mesh.AddNode(nodes.back());
    }
    return nodes;
}

static NodeList TetraNodes(ChMesh& mesh, bool quadratic) {
    double s = 0.1;
    std::vector<ChVector3d> c = {{0, 0, 0}, {s, 0, 0}, {0, s, 0}, {0, 0, s}};
    if (quadratic) {
        int e[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {3, 1}, {2, 3}};
        for (auto& ed : e)
            c.push_back((c[ed[0]] + c[ed[1]]) * 0.5);
    }
    NodeList nodes;
    for (auto& p : c) {
        nodes.push_back(chrono_types::make_shared<ChNodeFEAxyz>(p));
        mesh.AddNode(nodes.back());
    }
    return nodes;
}

class MassMatrixTest : public ::testing::Test {
  protected:
    void SetUp() override {
        mesh = chrono_types::make_shared<ChMesh>();

        auto mat = chrono_types::make_shared<ChContinuumElastic>();
        mat->SetYoungModulus(1e7);
        mat->SetPoissonRatio(0.3);
        mat->SetDensity(1000);
        mat->SetRayleighDampingAlpha(0.2);
        mat->SetRayleighDampingBeta(0.01);

        auto n4 = TetraNodes(*mesh, false);
        auto tet4 = chrono_types::make_shared<ChElementTetraCorot_4>();
        tet4->SetNodes(n4[0], n4[1], n4[2], n4[3]);
        tet4->SetMaterial(mat);
        mesh->AddElement(tet4);

        auto n10 = TetraNodes(*mesh, true);
        auto tet10 = chrono_types::make_shared<ChElementTetraCorot_10>();
        tet10->SetNodes(n10[0], n10[1], n10[2], n10[3], n10[4], n10[5], n10[6], n10[7], n10[8], n10[9]);
        tet10->SetMaterial(mat);
        mesh->AddElement(tet10);

        auto n8 = HexaNodes(*mesh, false);
        auto hex8 = chrono_types::make_shared<ChElementHexaCorot_8>();
        hex8->SetNodes(n8[0], n8[1], n8[2], n8[3], n8[4], n8[5], n8[6], n8[7]);
        hex8->SetMaterial(mat);
        mesh->AddElement(hex8);

        auto n20 = HexaNodes(*mesh, true);
        auto hex20 = chrono_types::make_shared<ChElementHexaCorot_20>();
        hex20->SetNodes(n20[0], n20[1], n20[2], n20[3], n20[4], n20[5], n20[6], n20[7], n20[8], n20[9], n20[10], n20[11], n20[12], n20[13], n20[14], n20[15], n20[16], n20[17],
                        n20[18], n20[19]);
        hex20->SetMaterial(mat);
        mesh->AddElement(hex20);

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(1e6, 0.3);
        auto shell_mat = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        shell_mat->SetDensity(100);
        NodeList nb;
        for (auto& p : {ChVector3d(0, 0, 0), ChVector3d(0.1, 0, 0), ChVector3d(0, 0, 0.1)}) {
            nb.push_back(chrono_types::make_shared<ChNodeFEAxyz>(p));
            mesh->AddNode(nb.back());
        }
        auto bst = chrono_types::make_shared<ChElementShellBST>();
        bst->SetNodes(nb[0], nb[1], nb[2], nullptr, nullptr, nullptr);
        bst->AddLayer(0.01, 0, shell_mat);
        mesh->AddElement(bst);

        sys.Add(mesh);
        sys.SetSolver(chrono_types::make_shared<ChSolverMINRES>());
        sys.DoStepDynamics(1e-6);  // performs the initial setup of all elements

        // Rigidly rotate and slightly deform all nodes, then update (corotational frames)
        ChQuaternion<> q = QuatFromAngleAxis(0.7, ChVector3d(1, 2, 3).GetNormalized());
        int i = 0;
        for (auto& node : mesh->GetNodes()) {
            auto n = std::static_pointer_cast<ChNodeFEAxyz>(node);
            ChVector3d p = q.Rotate(n->GetPos()) + ChVector3d(1, -2, 0.5);
            p += 0.002 * ChVector3d(std::sin(1.0 + i), std::cos(2.0 + i), std::sin(3.0 * i));
            n->SetPos(p);
            n->SetPosDt(ChVector3d(0.1 * i, -0.2, 0.3));
            i++;
        }
        sys.Update(UpdateFlags::UPDATE_ALL);
    }

    ChSystemSMC sys;
    std::shared_ptr<ChMesh> mesh;
};

TEST_F(MassMatrixTest, element_mass_matrix) {
    for (auto& el : mesh->GetElements()) {
        int n = (int)el->GetNumCoordsPosLevel();
        ChMatrixDynamic<> M(n, n);
        ChMatrixDynamic<> H(n, n);
        M.setConstant(-1);  // overwritten entirely
        el->ComputeMmatrixGlobal(M);
        el->ComputeKRMmatricesGlobal(H, 0, 0, 1);
        for (int r = 0; r < n; r++)
            for (int c = 0; c < n; c++)
                ASSERT_EQ(M(r, c), H(r, c)) << "element with " << n << " coordinates, entry (" << r << "," << c << ")";
        ASSERT_GT(M.trace(), 0);
    }
}

TEST_F(MassMatrixTest, mesh_Mv) {
    unsigned int nv = mesh->GetNumCoordsVelLevel();
    ASSERT_EQ(mesh->GetOffset_w(), 0u);
    ChVectorDynamic<> w(nv);
    for (unsigned int i = 0; i < nv; i++)
        w(i) = std::sin(0.3 * i + 0.1);

    // Reference: R = c * sum_e M_e * w_e with M_e from ComputeKRMmatricesGlobal(H, 0, 0, 1)
    double c = -0.75;
    ChVectorDynamic<> Rref(nv);
    Rref.setZero();
    for (auto& el : mesh->GetElements()) {
        int n = (int)el->GetNumCoordsPosLevel();
        ChMatrixDynamic<> H(n, n);
        el->ComputeKRMmatricesGlobal(H, 0, 0, 1);
        ChVectorDynamic<> we(n);
        for (unsigned int in = 0; in < el->GetNumNodes(); in++)
            we.segment(3 * in, 3) = w.segment(el->GetNode(in)->NodeGetOffsetVelLevel(), 3);
        ChVectorDynamic<> fe = c * H * we;
        for (unsigned int in = 0; in < el->GetNumNodes(); in++)
            Rref.segment(el->GetNode(in)->NodeGetOffsetVelLevel(), 3) += fe.segment(3 * in, 3);
    }
    // Nodal masses (zero here unless set on nodes)
    ChVectorDynamic<> Rnodes(nv);
    Rnodes.setZero();
    for (unsigned int j = 0; j < mesh->GetNumNodes(); j++)
        mesh->GetNode(j)->NodeIntLoadResidual_Mv(mesh->GetNode(j)->NodeGetOffsetVelLevel(), Rnodes, w, c);
    Rref += Rnodes;

    for (int nthreads : {1, 4}) {
        sys.SetNumThreads(nthreads);
        ChVectorDynamic<> R(nv);
        R.setZero();
        mesh->IntLoadResidual_Mv(mesh->GetOffset_w(), R, w, c);
        ASSERT_GT(R.norm(), 0);
        for (unsigned int i = 0; i < nv; i++)
            ASSERT_NEAR(R(i), Rref(i), 1e-14 * Rref.lpNorm<Eigen::Infinity>()) << "nthreads=" << nthreads;
    }
}
