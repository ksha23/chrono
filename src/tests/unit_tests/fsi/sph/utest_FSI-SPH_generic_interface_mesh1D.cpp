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
// Unit test for fluid forces on 1-D FEA meshes with the generic FSI interface.
//
// An ANCF cable, clamped at its lower end, stands in a periodic channel of fluid moving in +x. The same
// problem is run with the custom SPH FSI interface and with the generic FSI interface. Both interfaces
// obtain the nodal forces from the same fluid solver, so the forces applied to the cable nodes and the
// resulting cable motion must agree. The generic interface must also apply a nonzero drag to the cable.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
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

static CableResult RunCableInFlow(bool generic_interface, int num_steps) {
    const double spacing = 0.02;
    const double Lx = 0.4, Ly = 0.2, Lz = 0.3;
    const double flow_speed = 0.2;
    const double dt = 2e-4;

    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    sysMBS.SetSolver(solver);

    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH, generic_interface);
    sysFSI.SetVerbose(false);
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysFSI.SetStepSizeCFD(dt);
    sysFSI.SetStepsizeMBD(dt);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1e-3;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 1.0;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.artificial_viscosity = 0.02;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sysSPH.SetSPHParameters(sph_params);

    // Periodic in x and y, bottom wall. The bottom BCE plate has markers at x = -Lx/2 ... +Lx/2 (inclusive) with
    // the fluid spacing, so the periodic lengths are Lx + spacing and Ly + spacing.
    ChVector3d cMin(-Lx / 2 - spacing / 2, -Ly / 2 - spacing / 2, -10 * spacing);
    ChVector3d cMax(+Lx / 2 + spacing / 2, +Ly / 2 + spacing / 2, Lz + 10 * spacing);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), {BCType::PERIODIC, BCType::PERIODIC, BCType::NONE});

    // Fluid in hydrostatic equilibrium, moving in +x, starting one spacing above the first BCE layer
    double gz = 9.81;
    double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();
    int nx = (int)std::round(Lx / spacing) + 1;
    int ny = (int)std::round(Ly / spacing) + 1;
    int nz = (int)std::round(Lz / spacing);
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            for (int iz = 0; iz < nz; iz++) {
                ChVector3d pos(-Lx / 2 + ix * spacing, -Ly / 2 + iy * spacing, (iz + 1) * spacing);
                double p = sysSPH.GetDensity() * gz * (Lz + spacing / 2 - pos.z());
                double rho = sysSPH.GetDensity() + p / c2;
                sysSPH.AddSPHParticle(pos, rho, p, sysSPH.GetViscosity(), ChVector3d(flow_speed, 0, 0));
            }
        }
    }
    auto bottom_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(Lx, Ly, Lz), {0, 0, -1});
    sysFSI.AddFsiBoundary(bottom_bce, ChFrame<>(ChVector3d(0, 0, Lz / 2), QUNIT));

    // Vertical ANCF cable, clamped at the bottom
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.02);
    section->SetYoungModulus(1e6);
    section->SetDensity(1000);
    section->SetRayleighDamping(0.02);

    auto mesh = chrono_types::make_shared<ChMesh>();
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, 8, ChVector3d(0, 0, 0.22), ChVector3d(0, 0, 0.02));
    auto base_node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().back());
    auto tip_node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().front());
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
    const int num_steps = 50;
    auto custom = RunCableInFlow(false, num_steps);
    auto generic = RunCableInFlow(true, num_steps);

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

    // The fluid drags the cable downstream
    EXPECT_GT(total_custom.x(), 0);
    EXPECT_GT(total_generic.x(), 0);

    // Both interfaces apply the same fluid forces
    EXPECT_LE(max_diff, 1e-3 * max_force);
    EXPECT_LE((custom.tip_pos - generic.tip_pos).Length(), 1e-6);
}
