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
// Regression test that pins the results of short explicit (WCSPH) SPH runs.
//
// Each case runs a small CRM or CFD problem for a fixed number of steps and reduces the final state to
// a few scalars (mean particle speed, mean particle height, force on the solid), which are compared with
// reference values recorded with the solver at commit aa3922df4a. The cases cover:
//   - CRM with a kinematically driven wheel, the Adami boundary condition, and an active domain that
//     moves with the wheel (so the number of active particles changes during the run);
//   - CFD with a fixed submerged body and the Adami boundary condition;
//   - every particle shifting method, for both CRM and CFD.
//
// The tolerance is 2e-4 relative to the reference, or 2e-4 times a scale for small values (the forces in the
// CFD case are compared against a 1 N scale, i.e. to 2e-4 N). It is above the difference between GPU backends
// (CUDA and HIP agree to about 1e-6 relative, and to 4e-5 N on the small lateral force of the CFD case) and
// well below the change that dropping a neighbor interaction or mis-evaluating a shifting term produces (the
// shifting methods differ from each other by 5e-4 relative in mean speed and by 5e-3 N or more in force).
//
// Set the environment variable CHRONO_SPH_PRINT_REFERENCE to print the values in the format of the
// reference table.
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

namespace {

struct Result {
    double mean_speed;   // mean speed of the SPH particles
    double mean_height;  // mean z of the SPH particles
    double force_x;      // FSI force on the solid, x component
    double force_z;      // FSI force on the solid, z component
};

// Mean speed and height of the SPH particles.
void ParticleStats(ChFsiFluidSystemSPH& sysSPH, Result& r) {
    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();
    size_t n = sysSPH.GetNumFluidMarkers();
    double s = 0, z = 0;
    for (size_t i = 0; i < n; i++) {
        s += vel[i].Length();
        z += pos[i].z();
    }
    r.mean_speed = s / n;
    r.mean_height = z / n;
}

// CRM bed with a wheel driven along x at constant speed and spin (a fixed body whose state is set every step).
Result RunCrmWheel(ShiftingMethod shifting, bool active_domain, int num_steps) {
    double spacing = 0.01;
    double dt = 1e-4;
    double bx = 0.4, by = 0.2, bz = 0.08;
    double radius = 0.05, width = 0.06;
    double speed = 0.2, omega = 5.0;

    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    fsi.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.num_bce_layers = 3;
    sph_params.shifting_method = shifting;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_variable_time_step = false;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.num_proximity_search_steps = 1;
    fsi.SetSPHParameters(sph_params);
    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    fsi.Construct(ChVector3d(bx, by, bz), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);

    ChVector3d p0(-bx / 2 + 0.1, 0, bz + radius - 0.01);
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, ChVector3d(0, 1, 0), radius, width));
    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetPos(p0);
    wheel->SetMass(1);
    wheel->SetFixed(true);
    sysMBS.AddBody(wheel);
    fsi.AddRigidBody(wheel, geometry, true);
    if (active_domain)
        fsi.SetActiveDomain(ChVector3d(0.15));

    fsi.Initialize();

    double t = 0;
    for (int step = 0; step < num_steps; step++) {
        wheel->SetPos(p0 + ChVector3d(speed * t, 0, 0));
        wheel->SetRot(QuatFromAngleY(omega * t));
        wheel->SetPosDt(ChVector3d(speed, 0, 0));
        wheel->SetAngVelParent(ChVector3d(0, omega, 0));
        fsi.DoStepDynamics(dt);
        t += dt;
    }

    Result r;
    ParticleStats(*fsi.GetFluidSystemSPH(), r);
    r.force_x = fsi.GetFsiBodyForce(wheel).x();
    r.force_z = fsi.GetFsiBodyForce(wheel).z();
    return r;
}

// CFD tank with a fixed box submerged near the floor and side walls (so its markers neighbor the wall markers).
Result RunCfdTank(ShiftingMethod shifting, int num_steps) {
    double spacing = 0.01;
    double dt = 1e-4;
    double bx = 0.3, by = 0.12, bz = 0.15;
    double fz = 0.1;

    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1e-3;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.num_bce_layers = 3;
    sph_params.max_velocity = 2.0;
    sph_params.shifting_method = shifting;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.artificial_viscosity = 0.02;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.num_proximity_search_steps = 1;
    fsi.SetSPHParameters(sph_params);
    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Fluid in the left part of an open-top tank, so that it flows (a small dam break)
    fsi.Construct(ChVector3d(bx / 2, by, fz), ChVector3d(-bx / 4, 0, 0), BoxSide::NONE);
    fsi.AddBoxContainer(ChVector3d(bx, by, bz), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);

    // Submerged in the moving fluid, spanning the tank width and close to the floor
    ChVector3d box_size(0.04, by, 0.03);
    ChVector3d box_pos(-0.06, 0, box_size.z() / 2 + 0.02);
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, box_size));
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(box_pos);
    body->SetMass(1);
    body->SetFixed(true);
    sysMBS.AddBody(body);
    fsi.AddRigidBody(body, geometry, true);

    fsi.Initialize();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    Result r;
    ParticleStats(*fsi.GetFluidSystemSPH(), r);
    r.force_x = fsi.GetFsiBodyForce(body).x();
    r.force_z = fsi.GetFsiBodyForce(body).z();
    return r;
}

// Reference values recorded at commit aa3922df4a (CUDA, RTX 3090).
struct Reference {
    const char* name;
    Result value;
    Result scale;  // magnitude below which a value is compared absolutely
};

bool print_reference = std::getenv("CHRONO_SPH_PRINT_REFERENCE") != nullptr;

void Compare(const Reference& ref, const Result& r) {
    if (print_reference)
        printf("    {\"%s\", {%.9g, %.9g, %.9g, %.9g}, ...},\n", ref.name, r.mean_speed, r.mean_height, r.force_x, r.force_z);
    const double tol = 2e-4;
    auto check = [&](const char* what, double v, double v_ref, double scale) {
        double err = std::abs(v - v_ref);
        double allowed = tol * std::max(std::abs(v_ref), scale);
        EXPECT_LE(err, allowed) << ref.name << ": " << what << " = " << v << ", reference " << v_ref;
    };
    check("mean speed", r.mean_speed, ref.value.mean_speed, ref.scale.mean_speed);
    check("mean height", r.mean_height, ref.value.mean_height, ref.scale.mean_height);
    check("force x", r.force_x, ref.value.force_x, ref.scale.force_x);
    check("force z", r.force_z, ref.value.force_z, ref.scale.force_z);
}

const ShiftingMethod shifting_methods[] = {ShiftingMethod::NONE,      ShiftingMethod::XSPH,      ShiftingMethod::PPST,
                                           ShiftingMethod::PPST_XSPH, ShiftingMethod::DIFFUSION, ShiftingMethod::DIFFUSION_XSPH};

// Note: at this revision DIFFUSION_XSPH gives the same values as DIFFUSION, because the XSPH term is accumulated
// only for the XSPH and PPST_XSPH methods. The values pin the current behavior.
// clang-format off
const Reference crm_ad_ref = {"crm_wheel_active_domain", {0.00420214473, 0.0394309263, -1.70155406, 10.7757187}, {1e-3, 1e-2, 1e-1, 1e-1}};
const Reference crm_ref[] = {
    {"crm_wheel_NONE",           {0.0117543814, 0.0394445205, -0.75520575, 9.61086559}, {1e-3, 1e-2, 1e-1, 1e-1}},
    {"crm_wheel_XSPH",           {0.0117602223, 0.039444552, -0.700991213, 9.42092228}, {1e-3, 1e-2, 1e-1, 1e-1}},
    {"crm_wheel_PPST",           {0.0117569543, 0.0394452663, -0.764058113, 9.65379906}, {1e-3, 1e-2, 1e-1, 1e-1}},
    {"crm_wheel_PPST_XSPH",      {0.0117627959, 0.0394453029, -0.709759057, 9.46430302}, {1e-3, 1e-2, 1e-1, 1e-1}},
    {"crm_wheel_DIFFUSION",      {0.0117567022, 0.0394459875, -0.799972951, 9.78312588}, {1e-3, 1e-2, 1e-1, 1e-1}},
    {"crm_wheel_DIFFUSION_XSPH", {0.0117567022, 0.0394459875, -0.799972951, 9.78312588}, {1e-3, 1e-2, 1e-1, 1e-1}},
};
const Reference cfd_ref[] = {
    {"cfd_tank_NONE",            {0.0791358324, 0.0513446715, -0.0425862446, -1.76556993}, {1e-3, 1e-2, 1.0, 1.0}},
    {"cfd_tank_XSPH",            {0.0791946806, 0.0513443305, -0.0369288065, -1.77757812}, {1e-3, 1e-2, 1.0, 1.0}},
    {"cfd_tank_PPST",            {0.0791505268, 0.0513466746, -0.0378565565, -1.7796706}, {1e-3, 1e-2, 1.0, 1.0}},
    {"cfd_tank_PPST_XSPH",       {0.079208177, 0.0513462192, -0.0334973559, -1.78979242}, {1e-3, 1e-2, 1.0, 1.0}},
    {"cfd_tank_DIFFUSION",       {0.079112507, 0.0513497381, -0.0410678163, -1.76251602}, {1e-3, 1e-2, 1.0, 1.0}},
    {"cfd_tank_DIFFUSION_XSPH",  {0.079112507, 0.0513497381, -0.0410678163, -1.76251602}, {1e-3, 1e-2, 1.0, 1.0}},
};
// clang-format on

}  // namespace

TEST(SPHStepResults, CrmWheelActiveDomain) {
    Compare(crm_ad_ref, RunCrmWheel(ShiftingMethod::PPST_XSPH, true, 300));
}

TEST(SPHStepResults, CrmShifting) {
    for (int i = 0; i < 6; i++)
        Compare(crm_ref[i], RunCrmWheel(shifting_methods[i], false, 100));
}

TEST(SPHStepResults, CfdShifting) {
    for (int i = 0; i < 6; i++)
        Compare(cfd_ref[i], RunCfdTank(shifting_methods[i], 200));
}
