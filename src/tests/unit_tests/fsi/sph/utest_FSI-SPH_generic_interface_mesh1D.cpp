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
// Unit test for FSI forces on 1-D FEA meshes with the generic FSI interface.
//
// A column of CRM soil collapses onto an ANCF cable clamped at its base (a reduced version of
// demo_FSI-SPH_Flexible_Cable). The same problem is run with the custom SPH FSI interface and with
// the generic FSI interface. Both interfaces obtain the nodal forces from the same SPH solver, so the
// forces applied to the cable nodes and the resulting cable motion must agree. The comparison is made
// shortly after the soil reaches the cable, before GPU round-off differences between runs grow.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChMesh.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

struct CableResult {
    std::vector<ChVector3d> node_forces;  // FSI forces on the cable nodes after the last step
    ChVector3d tip_pos;                   // position of the free end of the cable
};

static CableResult RunCable(bool generic_interface, int num_steps) {
    const double spacing = 0.02;
    const double Lx = 1.6, Ly = 0.2, Hc = 1.0;  // container
    const double a = 0.6, H = 0.6;              // soil column
    const double dt = 2.5e-4;

    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysMBS.SetSolver(chrono_types::make_shared<ChSolverSparseQR>());

    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH, generic_interface);
    sysFSI.SetVerbose(false);
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysFSI.SetStepSizeCFD(dt);
    sysFSI.SetStepsizeMBD(dt);

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.5;
    mat_props.mu_fric_2 = 0.5;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    sysSPH.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.0;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.artificial_viscosity = 0.5;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sysSPH.SetSPHParameters(sph_params);

    // Container walls at x = -Lx/2, +Lx/2 and z = 0, periodic in y. The BCE plates have markers at
    // y = -Ly/2 ... +Ly/2 (inclusive) with the particle spacing, so the periodic length is Ly + spacing.
    ChVector3d cMin(-Lx, -Ly / 2 - spacing / 2, -0.5);
    ChVector3d cMax(+Lx, +Ly / 2 + spacing / 2, 2 * Hc);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Soil column against the left wall, with a lithostatic initial stress
    int nx = (int)std::round(a / spacing);
    int ny = (int)std::round(Ly / spacing) + 1;
    int nz = (int)std::round(H / spacing);
    double rho = sysSPH.GetDensity();
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            for (int iz = 0; iz < nz; iz++) {
                ChVector3d pos(-Lx / 2 + (ix + 1) * spacing, -Ly / 2 + iy * spacing, (iz + 1) * spacing);
                double p = rho * 9.81 * (H + spacing / 2 - pos.z());
                sysSPH.AddSPHParticle(pos, rho, p, sysSPH.GetViscosity(), VNULL, ChVector3d(-p), VNULL);
            }
        }
    }
    auto container_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(Lx, Ly, Hc), {2, 0, -1});
    sysFSI.AddFsiBoundary(container_bce, ChFrame<>(ChVector3d(0, 0, Hc / 2), QUNIT));

    // Vertical ANCF cable downstream of the soil column, clamped at the bottom
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.02);
    section->SetYoungModulus(6e8);
    section->SetDensity(8000);
    section->SetRayleighDamping(0.02);

    auto mesh = chrono_types::make_shared<ChMesh>();
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, 8, ChVector3d(0.1, 0, 0.5), ChVector3d(0.1, 0, 0.005));
    auto base_node = builder.GetLastBeamNodes().back();
    auto tip_node = builder.GetLastBeamNodes().front();
    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    pos_const->Initialize(base_node, ground);
    sysMBS.Add(pos_const);
    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    dir_const->Initialize(base_node, ground);
    dir_const->SetDirectionInAbsoluteCoords(base_node->GetSlope1());
    sysMBS.Add(dir_const);
    sysMBS.Add(mesh);

    auto fsi_mesh = sysFSI.AddFeaMesh1D(mesh, false);
    EXPECT_TRUE(fsi_mesh != nullptr);

    sysFSI.Initialize();
    for (int step = 0; step < num_steps; step++)
        sysFSI.DoStepDynamics(dt);

    CableResult r;
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i));
        r.node_forces.push_back(node->GetForce());
    }
    r.tip_pos = tip_node->GetPos();
    return r;
}

TEST(SPH_generic_interface, mesh1D_forces) {
    // The soil front reaches the cable at about t = 0.31 s; compare at t = 0.34 s
    const int num_steps = 1360;
    auto custom = RunCable(false, num_steps);
    auto generic = RunCable(true, num_steps);

    ASSERT_EQ(custom.node_forces.size(), generic.node_forces.size());

    ChVector3d total_custom = VNULL;
    ChVector3d total_generic = VNULL;
    double max_force = 0;
    double max_diff = 0;
    for (size_t i = 0; i < custom.node_forces.size(); i++) {
        total_custom += custom.node_forces[i];
        total_generic += generic.node_forces[i];
        max_force = std::max(max_force, custom.node_forces[i].Length());
        max_diff = std::max(max_diff, (custom.node_forces[i] - generic.node_forces[i]).Length());
    }

    std::cout << "  total cable force, custom interface:  " << total_custom << std::endl;
    std::cout << "  total cable force, generic interface: " << total_generic << std::endl;
    std::cout << "  max nodal force difference: " << max_diff << " (max nodal force " << max_force << ")" << std::endl;
    std::cout << "  cable tip, custom:  " << custom.tip_pos << std::endl;
    std::cout << "  cable tip, generic: " << generic.tip_pos << std::endl;

    // The soil pushes the cable downstream
    EXPECT_GT(total_custom.x(), 10.0);
    EXPECT_GT(total_generic.x(), 10.0);

    // Both interfaces apply the same fluid forces
    EXPECT_LE(max_diff, 1e-3 * max_force);
    EXPECT_LE((custom.tip_pos - generic.tip_pos).Length(), 1e-5);
}
