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
// Unit test for the selection of the consistent gradient and Laplacian
// discretization in the WCSPH CFD right-hand side (CfdCalcRHS_D).
//
// The consistent operators take effect only if BOTH options are enabled; with
// only one of them enabled, the correction matrix that is computed is never
// used. The kernel is therefore instantiated for exactly two cases, and this
// test pins that choice on a small 3D dam break:
//
// 1. Two runs with both options off are bitwise identical (the comparisons
//    below are meaningful only if a run is deterministic).
// 2. Enabling only the gradient option, or only the Laplacian option, gives
//    results bitwise identical to both off.
// 3. Enabling both options gives different results (the consistent path is
//    still reachable), which remain finite and inside the container.
//
// =============================================================================

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

int num_failures = 0;

void Check(const std::string& name, bool cond) {
    if (cond) {
        std::cout << "  ok:   " << name << std::endl;
    } else {
        std::cout << "  FAIL: " << name << std::endl;
        num_failures++;
    }
}

// Container interior dimensions and fluid column dimensions
const double spacing = 0.01;
const double cxDim = 0.30;
const double cyDim = 0.10;
const double czDim = 0.20;
const double fxDim = 0.10;
const double fzDim = 0.10;

struct State {
    std::vector<ChVector3d> pos;
    std::vector<ChVector3d> vel;
    std::vector<ChVector3d> props;  // density, pressure, viscosity
};

State RunDamBreak(bool consistent_gradient, bool consistent_laplacian) {
    double dt = 1e-4;
    int num_steps = 200;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 2.0;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.eos_type = EosType::TAIT;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.use_consistent_gradient_discretization = consistent_gradient;
    sph_params.use_consistent_laplacian_discretization = consistent_laplacian;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Fluid column against the X_NEG wall, inside a closed-bottom container
    fsi.Construct(ChVector3d(fxDim, cyDim, fzDim), ChVector3d(-cxDim / 2 + fxDim / 2, 0, 0), BoxSide::NONE);
    fsi.AddBoxContainer(ChVector3d(cxDim, cyDim, czDim), ChVector3d(0, 0, 0), BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS | BoxSide::Y_NEG | BoxSide::Y_POS);

    fsi.Initialize();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    State state;
    size_t num_sph = fsi.GetNumSPHParticles();
    state.pos = sysSPH->GetParticlePositions();
    state.vel = sysSPH->GetParticleVelocities();
    state.props = sysSPH->GetParticleFluidProperties();
    state.pos.resize(num_sph);
    state.vel.resize(num_sph);
    state.props.resize(num_sph);

    return state;
}

bool BitwiseEqual(const std::vector<ChVector3d>& a, const std::vector<ChVector3d>& b) {
    if (a.size() != b.size())
        return false;
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(ChVector3d)) == 0;
}

bool BitwiseEqual(const State& a, const State& b) {
    return BitwiseEqual(a.pos, b.pos) && BitwiseEqual(a.vel, b.vel) && BitwiseEqual(a.props, b.props);
}

double MaxDifference(const std::vector<ChVector3d>& a, const std::vector<ChVector3d>& b) {
    double diff = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); i++)
        diff = std::max(diff, (a[i] - b[i]).Length());
    return diff;
}

int main(int argc, char* argv[]) {
    std::cout << "Small 3D dam break, " << (int)std::round(fxDim / spacing) << "x" << (int)std::round(cyDim / spacing) << "x" << (int)std::round(fzDim / spacing) << " fluid column"
              << std::endl;

    State ff = RunDamBreak(false, false);
    State ff2 = RunDamBreak(false, false);
    State tf = RunDamBreak(true, false);
    State ft = RunDamBreak(false, true);
    State tt = RunDamBreak(true, true);

    size_t num_sph = ff.pos.size();
    double xmax = -1e9;
    for (const auto& p : ff.pos)
        xmax = std::max(xmax, p.x());

    std::cout << "  SPH particles: " << num_sph << std::endl;
    std::cout << "  dam front (both off): x = " << xmax << " (initial " << -cxDim / 2 + fxDim << ")" << std::endl;
    std::cout << "  max |dpos| gradient only vs off:  " << MaxDifference(tf.pos, ff.pos) << std::endl;
    std::cout << "  max |dpos| Laplacian only vs off: " << MaxDifference(ft.pos, ff.pos) << std::endl;
    std::cout << "  max |dpos| both on vs off:        " << MaxDifference(tt.pos, ff.pos) << std::endl;

    // Guards against the comparisons below passing on an empty or static state
    Check("the fluid column is populated", num_sph > 500);
    Check("the dam front has started to move", xmax > -cxDim / 2 + fxDim + 0.1 * spacing);

    Check("two runs with both options off are bitwise identical", BitwiseEqual(ff, ff2));
    Check("gradient option alone gives the same result as both off", BitwiseEqual(tf, ff));
    Check("Laplacian option alone gives the same result as both off", BitwiseEqual(ft, ff));

    Check("both options on gives a different result", !BitwiseEqual(tt, ff));

    bool finite = tt.pos.size() == num_sph;
    bool inside = finite;
    for (size_t i = 0; i < tt.pos.size(); i++) {
        const auto& p = tt.pos[i];
        const auto& v = tt.vel[i];
        finite = finite && std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) && std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        inside = inside && std::abs(p.x()) < cxDim / 2 + spacing && p.z() > -spacing && p.z() < czDim;
    }
    Check("both options on: state is finite", finite);
    Check("both options on: fluid stays inside the container", inside);

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll consistent-discretization selection checks passed" << std::endl;
    return 0;
}
