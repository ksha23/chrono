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
// Authors: Chrono contributors
// =============================================================================
//
// Test that the vertex normals of the SCM visualization mesh stay consistent with
// the deformed mesh: after each step, every vertex normal must equal the average of
// the unit normals of its incident faces, including at vertices neighboring the
// deformed region and at vertices on the boundary of the patch.
//
// =============================================================================

#include <cmath>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

// Compare the mesh normals against normals recomputed from scratch.
// Return the number of non-finite normals and the maximum deviation over all finite normals.
static void CheckNormals(const SCMTerrain& terrain, int& num_nan, double& max_err) {
    const auto& mesh = *terrain.GetMesh()->GetMesh();
    const auto& vertices = mesh.GetCoordsVertices();
    const auto& normals = mesh.GetCoordsNormals();
    const auto& faces = mesh.GetIndicesNormals();

    std::vector<ChVector3d> ref(vertices.size(), VNULL);
    std::vector<int> count(vertices.size(), 0);
    for (const auto& f : faces) {
        ChVector3d nrm = Vcross(vertices[f[1]] - vertices[f[0]], vertices[f[2]] - vertices[f[0]]);
        nrm.Normalize();
        for (int k = 0; k < 3; k++) {
            ref[f[k]] += nrm;
            count[f[k]]++;
        }
    }

    num_nan = 0;
    max_err = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
        const auto& n = normals[i];
        if (!std::isfinite(n.x()) || !std::isfinite(n.y()) || !std::isfinite(n.z())) {
            num_nan++;
            continue;
        }
        max_err = std::max(max_err, (n - ref[i] / count[i]).Length());
    }
}

// A fixed box pressed into the soil across the +x edge of the patch deforms interior and boundary nodes.
TEST(SCMTerrain, mesh_normals_contact) {
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    SCMTerrain terrain(&sys);
    terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4);
    terrain.Initialize(2.0, 2.0, 0.05);
    terrain.SetMeshWireframe(false);

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto box = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.3, 0.2, 1000, true, true, mat);
    box->SetPos(ChVector3d(1.0, 0.1, 0.1 - 0.03));  // straddles the x = 1 edge, 3 cm into the soil
    box->SetFixed(true);
    sys.AddBody(box);

    for (int i = 0; i < 5; i++) {
        sys.DoStepDynamics(1e-3);
        box->SetPos(box->GetPos() - ChVector3d(0.01, 0, 0.005));  // move the box and push it deeper
    }

    ASSERT_GT(terrain.GetNumRayHits(), 0);

    int num_nan;
    double max_err;
    CheckNormals(terrain, num_nan, max_err);
    std::cout << "contact: non-finite normals = " << num_nan << "  max normal error = " << max_err << std::endl;
    EXPECT_EQ(num_nan, 0);
    EXPECT_LT(max_err, 1e-12);
}

// Nodes set through SetModifiedNodes (a pit) must also leave consistent normals, including on the pit rim.
TEST(SCMTerrain, mesh_normals_set_nodes) {
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    SCMTerrain terrain(&sys);
    terrain.Initialize(2.0, 2.0, 0.05);
    terrain.SetMeshWireframe(false);

    std::vector<SCMTerrain::NodeLevel> nodes;
    for (int i = -5; i <= 5; i++)
        for (int j = -5; j <= 5; j++)
            nodes.push_back(std::make_pair(ChVector2i(i, j), -0.1 + 0.002 * (i * i + j * j)));
    for (int j = -20; j <= 20; j++)
        nodes.push_back(std::make_pair(ChVector2i(20, j), -0.05));  // along the +x boundary
    terrain.SetModifiedNodes(nodes);

    int num_nan;
    double max_err;
    CheckNormals(terrain, num_nan, max_err);
    std::cout << "set nodes: non-finite normals = " << num_nan << "  max normal error = " << max_err << std::endl;
    EXPECT_EQ(num_nan, 0);
    EXPECT_LT(max_err, 1e-12);
}
