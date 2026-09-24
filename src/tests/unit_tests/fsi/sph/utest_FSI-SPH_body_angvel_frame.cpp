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
// Unit test for the rigid-body BCE marker velocities in Chrono::FSI-SPH.
//
// The BCE markers of a rigid FSI body must move with the rigid-body velocity field
//     v_marker = v_body + w x (x_marker - x_body)
// with w the body angular velocity in the global frame. A box body is given an initial linear and
// angular velocity, with its orientation chosen such that the angular velocity expressed in the global
// frame and in the body frame differ. The BCE marker velocities are compared with the analytic field
// after initialization of the FSI system (which loads the solid states through the generic path for
// both interfaces) and after several coupled steps, for both the custom SPH FSI interface and the
// generic FSI interface. A body whose frame is aligned with the global frame (for which the two
// representations of the angular velocity coincide) is included as a control.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Maximum error between BCE marker velocities and the rigid-body velocity field of the current body state,
// relative to the largest rigid-body velocity over the markers.
static double MaxError(ChFsiFluidSystemSPH& sysSPH, const ChBody& body, const std::string& label) {
    // Markers are stored as fluid, boundary, rigid-body BCE
    size_t start = sysSPH.GetNumFluidMarkers() + sysSPH.GetNumBoundaryMarkers();
    size_t num_rigid = sysSPH.GetNumRigidBodyMarkers();
    EXPECT_GT(num_rigid, 0u);

    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();
    EXPECT_GE(pos.size(), start + num_rigid);

    const ChVector3d& x_body = body.GetPos();
    const ChVector3d& v_body = body.GetPosDt();
    ChVector3d w_global = body.GetAngVelParent();

    double max_err = 0;
    double max_vel = 0;
    for (size_t i = start; i < start + num_rigid; i++) {
        ChVector3d v_exact = v_body + Vcross(w_global, pos[i] - x_body);
        max_err = std::max(max_err, (vel[i] - v_exact).Length());
        max_vel = std::max(max_vel, v_exact.Length());
    }

    std::cout << "  " << label << ": " << num_rigid << " BCE markers, max |v_bce - v_exact| = " << max_err << " (max |v_exact| = " << max_vel << ")" << std::endl;

    return max_err / max_vel;
}

struct BceVelocityErrors {
    double initial;  // after initialization of the FSI system
    double stepped;  // after several coupled steps
};

static BceVelocityErrors BceVelocityError(bool generic_interface, const ChQuaterniond& rot, const ChVector3d& w_global) {
    const double spacing = 0.02;

    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(VNULL);

    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH, generic_interface);
    sysFSI.SetVerbose(false);
    sysFSI.SetGravitationalAcceleration(VNULL);
    sysFSI.SetStepSizeCFD(1e-4);
    sysFSI.SetStepsizeMBD(1e-4);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1e-3;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.initial_spacing = spacing;
    sysSPH.SetSPHParameters(sph_params);

    sysSPH.SetComputationalDomain(ChAABB(ChVector3d(-1, -1, -1), ChVector3d(1, 1, 1)), BC_NONE);

    // A small block of fluid, away from the body
    sysSPH.AddBoxSPH(ChVector3d(0, 0, -0.5), ChVector3d(0.1, 0.1, 0.1));

    // Rigid box body with BCE markers in its interior, spinning and translating
    const ChVector3d body_pos(0.1, -0.05, 0.3);
    const ChVector3d v_body(0.3, -0.2, 0.1);
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(body_pos);
    body->SetRot(rot);
    body->SetPosDt(v_body);
    body->SetAngVelParent(w_global);
    body->SetMass(1);
    body->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
    sysMBS.AddBody(body);

    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, ChVector3d(0.3, 0.1, 0.06)));
    sysFSI.AddRigidBody(body, geometry, false);

    sysFSI.Initialize();

    std::string label = std::string(generic_interface ? "generic" : "custom ") + " interface";
    BceVelocityErrors err;
    err.initial = MaxError(sysSPH, *body, label + ", after initialization");

    // The fluid is out of reach and the inertia is isotropic, so the body keeps spinning at constant angular velocity
    for (int step = 0; step < 10; step++)
        sysFSI.DoStepDynamics(1e-4);
    err.stepped = MaxError(sysSPH, *body, label + ", after 10 steps       ");

    return err;
}

// Single-precision BCE marker data
const double tolerance = 1e-5;

// Rotation about the global z axis, with the body frame rotated by 90 degrees about x
TEST(SPH_body_angvel_frame, rotated_body_custom) {
    auto err = BceVelocityError(false, QuatFromAngleX(CH_PI_2), ChVector3d(0, 0, 2));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}
TEST(SPH_body_angvel_frame, rotated_body_generic) {
    auto err = BceVelocityError(true, QuatFromAngleX(CH_PI_2), ChVector3d(0, 0, 2));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}

// Rotation about an oblique axis, with a general body orientation
TEST(SPH_body_angvel_frame, oblique_axis_custom) {
    ChQuaterniond rot = QuatFromAngleZ(0.4) * QuatFromAngleY(-0.7) * QuatFromAngleX(1.1);
    auto err = BceVelocityError(false, rot, ChVector3d(1.0, -2.0, 1.5));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}
TEST(SPH_body_angvel_frame, oblique_axis_generic) {
    ChQuaterniond rot = QuatFromAngleZ(0.4) * QuatFromAngleY(-0.7) * QuatFromAngleX(1.1);
    auto err = BceVelocityError(true, rot, ChVector3d(1.0, -2.0, 1.5));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}

// Body frame aligned with the global frame (global and local angular velocities coincide)
TEST(SPH_body_angvel_frame, aligned_body_custom) {
    auto err = BceVelocityError(false, QUNIT, ChVector3d(1.0, -2.0, 1.5));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}
TEST(SPH_body_angvel_frame, aligned_body_generic) {
    auto err = BceVelocityError(true, QUNIT, ChVector3d(1.0, -2.0, 1.5));
    EXPECT_LT(err.initial, tolerance);
    EXPECT_LT(err.stepped, tolerance);
}
