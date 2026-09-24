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
// Regression test that pins the result of short SPH runs through the WCSPH
// right-hand-side kernels (CrmCalcRHS_D, CfdCalcRHS_D), the boundary kernels
// (Adami and Holmes, CRM and CFD, with the Tait inverse equation of state) and
// the CRM stress update (mu(I) and MCC).
//
// Each case runs a small problem for a fixed number of steps and compares a few
// aggregate quantities of the SPH particles (mean position, mean speed, mean
// density and pressure, front position) against reference values recorded with
// the default single-precision build (CUDA, RTX 5060 Ti). The comparison uses a
// tolerance relative to a problem scale, not a bitwise one: changes that only
// affect rounding (a different GPU, FMA contraction, evaluating a constant in
// float instead of double) must stay inside it, while a change to the
// discretization or the constitutive model should not.
//
// The reference values apply to the default single-precision build only. With
// CH_USE_SPH_DOUBLE the values are printed but not checked.
//
// This test checks results, not the generated code. It does not detect double
// precision literals or calls that slow down the single-precision kernels
// without changing their results beyond rounding.
//
// Sensitivity, checked by mutating SphForceWCSPH.cu when the Holmes cases were
// added: removing the CFD delta-SPH term, halving the CRM or CFD artificial
// viscosity, or changing the Holmes velocity extrapolation limit makes the test
// fail. Removing the CRM tensile-instability term or the Jaumann rotation terms
// of the diagonal stress rate does not. The first is not active in these
// cohesionless cases (the mu(I) update clamps the pressure at zero, and in the
// MCC case the term moves the results by about 1e-5 of scale); the second moves
// the checked quantities by less than 0.11% of their scale.
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Aggregate quantities over the SPH (non-BCE) particles.
struct Checksum {
    size_t n = 0;
    double mean_x = 0, mean_y = 0, mean_z = 0;
    double max_x = 0;
    double mean_r = 0;  // mean horizontal distance from the z axis
    double mean_speed = 0;
    double mean_rho = 0;
    double mean_p = 0;
};

Checksum Compute(ChFsiFluidSystemSPH& sysSPH) {
    Checksum c;
    size_t n = sysSPH.GetNumFluidMarkers();
    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();
    auto prop = sysSPH.GetParticleFluidProperties();
    c.n = n;
    c.max_x = -1e30;
    for (size_t i = 0; i < n; i++) {
        c.mean_x += pos[i].x();
        c.mean_y += pos[i].y();
        c.mean_z += pos[i].z();
        c.max_x = std::max(c.max_x, pos[i].x());
        c.mean_r += std::sqrt(pos[i].x() * pos[i].x() + pos[i].y() * pos[i].y());
        c.mean_speed += vel[i].Length();
        c.mean_rho += prop[i].x();
        c.mean_p += prop[i].y();
    }
    c.mean_x /= n;
    c.mean_y /= n;
    c.mean_z /= n;
    c.mean_r /= n;
    c.mean_speed /= n;
    c.mean_rho /= n;
    c.mean_p /= n;
    return c;
}

// Mean pressure of the SPH particles.
double MeanPressure(ChFsiFluidSystemSPH& sysSPH) {
    size_t n = sysSPH.GetNumFluidMarkers();
    auto prop = sysSPH.GetParticleFluidProperties();
    double p = 0;
    for (size_t i = 0; i < n; i++)
        p += prop[i].y();
    return p / n;
}

void Print(const std::string& name, const Checksum& c) {
    printf("%s: n=%zu mean_x=%.9g mean_y=%.9g mean_z=%.9g max_x=%.9g mean_r=%.9g mean_speed=%.9g mean_rho=%.9g mean_p=%.9g\n", name.c_str(), c.n, c.mean_x, c.mean_y, c.mean_z,
           c.max_x, c.mean_r, c.mean_speed, c.mean_rho, c.mean_p);
}

int num_failures = 0;

// Check |val - ref| <= rtol * scale, where scale is a problem-specific magnitude for the quantity
// (so that quantities that are near zero, such as a mean coordinate of a symmetric body, are not
// compared with a relative tolerance on a tiny number).
void Check(const std::string& what, double val, double ref, double scale, double rtol) {
    double err = std::abs(val - ref);
    bool ok = err <= rtol * scale;
    printf("  %-26s %16.9g  ref %16.9g  |diff|/scale %.3e  (tol %.1e)  %s\n", what.c_str(), val, ref, err / scale, rtol, ok ? "ok" : "FAIL");
    if (!ok)
        num_failures++;
}

// -----------------------------------------------------------------------------
// Case 1: CRM granular column collapse (mu(I) rheology).
// A cylinder of soil is released inside an open box and spreads under gravity.
// -----------------------------------------------------------------------------
Checksum RunColumnCollapse(int num_steps, BoundaryMethod boundary_method) {
    double step_size = 1e-4;
    double spacing = 0.005;
    double radius = 0.05;
    double height = 0.1;
    double bxDim = 0.4, byDim = 0.4, bzDim = 0.15;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);
    sysFSI.SetVerbose(false);
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1500;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.3819;
    mat_props.mu_fric_2 = 0.3819;
    mat_props.average_diam = 0.002;
    sysSPH.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.artificial_viscosity = 0.5;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 2.4;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_variable_time_step = false;
    sph_params.kernel_type = KernelType::WENDLAND;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = boundary_method;
    sysSPH.SetSPHParameters(sph_params);

    ChVector3d cMin(-bxDim / 2 - 3 * spacing, -byDim / 2 - 3 * spacing, -bzDim - 3 * spacing);
    ChVector3d cMax(bxDim / 2 + 3 * spacing, byDim / 2 + 3 * spacing, 2 * bzDim + 3 * spacing);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    chrono::utils::ChGridSampler<> sampler(spacing);
    ChVector3d center(0.0, 0.0, -bzDim / 2 + height / 2 + 2 * spacing);
    auto points = sampler.SampleCylinderZ(center, radius, height / 2);
    double gz = 9.81;
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-(p.z() + height / 2) + height);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    auto box = chrono_types::make_shared<ChBody>();
    box->SetFixed(true);
    box->EnableCollision(false);
    sysMBS.AddBody(box);
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), {0, 0, -1});
    sysFSI.AddRigidBody(box, box_bce, ChFrame<>(ChVector3d(0, 0, 0), QUNIT), false);

    sysFSI.Initialize();
    for (int i = 0; i < num_steps; i++)
        sysFSI.DoStepDynamics(step_size);
    return Compute(sysSPH);
}

// -----------------------------------------------------------------------------
// Case 2: CFD dam break (weakly compressible fluid, Tait EOS, delta-SPH).
// A water column at the left end of a tank periodic in y collapses and runs along the floor.
// -----------------------------------------------------------------------------
Checksum RunDamBreak(int num_steps, BoundaryMethod boundary_method) {
    double step_size = 1e-4;
    double spacing = 0.01;
    double bxDim = 0.6, byDim = 0.1, bzDim = 0.4;
    double fxDim = 0.2, fyDim = 0.1, fzDim = 0.3;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetVerbose(false);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 5;
    sysSPH.SetCfdSPH(fluid_props);
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 3.0;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.artificial_viscosity = 0.03;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.kernel_type = KernelType::WENDLAND;
    sph_params.boundary_method = boundary_method;
    sysSPH.SetSPHParameters(sph_params);

    ChVector3d cMin(-bxDim / 2 - 10 * spacing, -byDim / 2 - spacing / 2, -2 * bzDim);
    ChVector3d cMax(+bxDim / 2 + 10 * spacing, +byDim / 2 + spacing / 2, +2 * bzDim);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - spacing, fyDim / 2, fzDim / 2 - spacing);
    chrono::utils::ChGridSampler<> sampler(spacing);
    auto points = sampler.SampleBox(boxCenter, boxHalfDim);
    double gz = 9.8;
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity());
    }

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sysMBS.AddBody(ground);
    auto ground_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), {2, 0, 2});
    sysFSI.AddFsiBoundary(ground_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT));

    sysFSI.Initialize();

    // The instantaneous mean pressure of a weakly compressible fluid carries acoustic oscillations that
    // amplify rounding differences, so the pressure checksum is averaged over the last 500 steps.
    int num_avg_steps = 500;
    int num_avg_samples = 0;
    double mean_p = 0;
    for (int i = 0; i < num_steps; i++) {
        sysFSI.DoStepDynamics(step_size);
        if (i >= num_steps - num_avg_steps && i % 10 == 0) {
            mean_p += MeanPressure(sysSPH);
            num_avg_samples++;
        }
    }
    auto c = Compute(sysSPH);
    c.mean_p = mean_p / num_avg_samples;
    return c;
}

// -----------------------------------------------------------------------------
// Case 3: CRM bed with the modified Cam-Clay rheology, a sphere dropped onto it.
// -----------------------------------------------------------------------------
Checksum RunMccSphereDrop(int num_steps, double& sphere_z) {
    double step_size = 1e-4;
    double spacing = 0.01;
    double bxDim = 0.3, byDim = 0.3, bzDim = 0.1;
    double sphere_radius = 0.04;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.8;
    mat_props.mu_fric_2 = 0.8;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    mat_props.rheology_model = RheologyCRM::MCC;
    mat_props.mcc_M = 1.2;
    mat_props.mcc_kappa = 0.01;
    mat_props.mcc_lambda = 0.1;
    mat_props.mcc_v_lambda = 2.0;
    fsi.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1;
    sph_params.artificial_viscosity = 0.5;
    sph_params.kernel_type = KernelType::WENDLAND;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    auto sphere = chrono_types::make_shared<ChBody>();
    double mass = 2.0;
    sphere->SetMass(mass);
    sphere->SetInertiaXX(ChVector3d(0.4 * mass * sphere_radius * sphere_radius));
    sphere->SetPos(ChVector3d(0, 0, bzDim + sphere_radius + 0.005));
    sphere->SetPosDt(ChVector3d(0, 0, -1.0));
    sysMBS.AddBody(sphere);
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere_radius, 0));
    fsi.AddRigidBody(sphere, geometry, true);

    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(bzDim));
    fsi.Construct(ChVector3d(bxDim, byDim, bzDim), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);
    fsi.Initialize();

    for (int i = 0; i < num_steps; i++)
        fsi.DoStepDynamics(step_size);
    sphere_z = sphere->GetPos().z();
    return Compute(*fsi.GetFluidSystemSPH());
}

int main(int argc, char* argv[]) {
#ifdef CHRONO_SPH_USE_DOUBLE
    bool check = false;
#else
    bool check = true;
#endif
    // Tolerances, relative to a problem scale for each quantity. Kinematic quantities are compared at
    // 0.2% and mean pressures at 0.5% of their scale. Rounding-only changes measured when these values
    // were recorded (FMA contraction disabled in the SPH kernels; the HIP backend on an AMD GPU instead of
    // CUDA) moved every quantity by less than 0.06% of its scale.
    const double rtol = 2e-3;
    const double rtol_p = 5e-3;

    // Case 1
    auto c1 = RunColumnCollapse(1000, BoundaryMethod::ADAMI);
    Print("column_collapse", c1);
    if (check) {
        Check("n", (double)c1.n, 6657, 1, 0);
        Check("mean_z", c1.mean_z, -0.0371872676, 0.1, rtol);
        Check("mean_r", c1.mean_r, 0.0409302454, 0.05, rtol);
        Check("mean_speed", c1.mean_speed, 0.299156503, 0.3, rtol);
        Check("mean_p", c1.mean_p, 359.184975, 700, rtol_p);
    }

    // Case 2
    auto c2 = RunDamBreak(2000, BoundaryMethod::ADAMI);
    Print("dam_break", c2);
    if (check) {
        Check("n", (double)c2.n, 6061, 1, 0);
        Check("mean_x", c2.mean_x, -0.132608381, 0.3, rtol);
        Check("mean_z", c2.mean_z, 0.0809445538, 0.15, rtol);
        Check("max_x", c2.max_x, 0.184732482, 0.3, rtol);
        Check("mean_speed", c2.mean_speed, 0.885350075, 1, rtol);
        Check("mean_p", c2.mean_p, 1129.35787, 1500, rtol_p);
    }

    // Case 3
    double sphere_z = 0;
    auto c3 = RunMccSphereDrop(500, sphere_z);
    Print("mcc_sphere_drop", c3);
    printf("mcc_sphere_drop: sphere_z=%.9g\n", sphere_z);
    if (check) {
        Check("mean_z", c3.mean_z, 0.0486123123, 0.05, rtol);
        Check("mean_speed", c3.mean_speed, 0.0405264516, 0.05, rtol);
        Check("mean_p", c3.mean_p, 1417.19448, 1700, rtol_p);
        Check("sphere_z", sphere_z, 0.10387144, 0.05, rtol);
    }

    // Cases 1 and 2 with Holmes boundary conditions (CrmHolmesBC_D, CfdHolmesBC_D)
    auto c4 = RunColumnCollapse(1000, BoundaryMethod::HOLMES);
    Print("column_collapse_holmes", c4);
    if (check) {
        Check("n", (double)c4.n, 6657, 1, 0);
        Check("mean_z", c4.mean_z, -0.0372279816, 0.1, rtol);
        Check("mean_r", c4.mean_r, 0.0409697402, 0.05, rtol);
        Check("mean_speed", c4.mean_speed, 0.300377554, 0.3, rtol);
        Check("mean_p", c4.mean_p, 346.116012, 700, rtol_p);
    }

    auto c5 = RunDamBreak(2000, BoundaryMethod::HOLMES);
    Print("dam_break_holmes", c5);
    if (check) {
        Check("n", (double)c5.n, 6061, 1, 0);
        Check("mean_x", c5.mean_x, -0.13198383, 0.3, rtol);
        Check("mean_z", c5.mean_z, 0.080445667, 0.15, rtol);
        Check("max_x", c5.max_x, 0.188554555, 0.3, rtol);
        Check("mean_speed", c5.mean_speed, 0.897619361, 1, rtol);
        Check("mean_p", c5.mean_p, 1147.74812, 1500, rtol_p);
    }

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "\nAll SPH kernel checksums within tolerance" << std::endl;
    return 0;
}
