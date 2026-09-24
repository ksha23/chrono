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
// Authors: Simulation-Based Engineering Laboratory, University of Wisconsin-Madison
// =============================================================================
//
// Regression test for the Jacobi solves of the implicit SPH (ISPH) solver.
//
// The V* and pressure Jacobi loops evaluate their stopping test on the device after every iteration, and the
// host reads it back only every LinSolverParameters::check_interval iterations. Iterations launched after the
// test is met must do nothing, so the solution may not depend on the check interval: a loop that simply
// checked less often would overshoot convergence by up to check_interval - 1 iterations and change the result.
//
// The test runs a small circular Couette cell (as in demo_FSI-SPH_CouetteFlow, coarser) for a few steps with
// several check intervals and three pressure tolerances: atol = 0 (every solve runs max_num_iters), a huge atol
// (every pressure solve stops after the minimum of 3 iterations) and an intermediate atol (solves stop after a
// varying number of iterations). For each tolerance the particle state must be bitwise identical across check
// intervals. The tolerances must also give different states (so the early exit is exercised), and the flow must
// stay finite with speeds below twice the free-fall speed over the simulated time plus the wall speed.
//
// The per-step iteration counts are read from the verbose solver output and pinned independently of the check
// interval: every pressure solve takes exactly max_num_iters iterations with atol = 0 and exactly 3 with the huge
// atol (the same counts as the host-side loop that the device-side test replaced), the intermediate atol stops
// at least once at a count that is not a multiple of the check interval, and the count sequences for all check
// intervals equal those for check_interval = 1.
//
// Only fluid and fixed-boundary markers are compared: the BCE markers of the rotating cylinder carry a ~1e-14
// out-of-plane velocity from the multibody side that is not reproducible from run to run.
//
// =============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
    #include <io.h>
#else
    #include <unistd.h>
#endif

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::endl;

const double initial_spacing = 0.25;  // mm
const double step_size = 5e-4;
const int num_steps = 20;
const int max_num_iters = 100;
const double omega = 62.0 / 60;
const double outer_radius = 5.0;
const double inner_radius = 3.8;
const double gravity = 9810.0;  // mm/s2

int num_failures = 0;

void check(bool condition, const std::string& what) {
    cout << (condition ? "  PASS  " : "  FAIL  ") << what << endl;
    if (!condition)
        num_failures++;
}

// Position, velocity, and (density, pressure, viscosity) of one marker
typedef std::array<double, 9> Record;

// Fluid and fixed-boundary marker records, sorted so that the comparison does not depend on output order
struct State {
    std::vector<Record> records;
    double vmax = 0;
    bool finite = true;
    std::vector<int> vstar_iters;     // V* Jacobi iterations, one per step
    std::vector<int> pressure_iters;  // pressure Jacobi iterations, one per step

    bool operator==(const State& other) const {
        return records.size() == other.records.size() && std::memcmp(records.data(), other.records.data(), records.size() * sizeof(Record)) == 0;
    }
};

// Redirect stdout (file descriptor level, so that printf output is included) to a temporary file.
class StdoutCapture {
  public:
    StdoutCapture() {
        std::fflush(stdout);
        cout.flush();
        m_file = std::tmpfile();
#ifdef _WIN32
        m_saved = _dup(_fileno(stdout));
        _dup2(_fileno(m_file), _fileno(stdout));
#else
        m_saved = dup(fileno(stdout));
        dup2(fileno(m_file), fileno(stdout));
#endif
    }

    // Restore stdout and return the captured text.
    std::string Finish() {
        std::fflush(stdout);
        cout.flush();
#ifdef _WIN32
        _dup2(m_saved, _fileno(stdout));
        _close(m_saved);
#else
        dup2(m_saved, fileno(stdout));
        close(m_saved);
#endif
        std::string text;
        std::rewind(m_file);
        char buf[4096];
        size_t len;
        while ((len = std::fread(buf, 1, sizeof(buf), m_file)) > 0)
            text.append(buf, len);
        std::fclose(m_file);
        return text;
    }

  private:
    FILE* m_file;
    int m_saved;
};

// Iteration counts ("#Iter=N") of all verbose output lines that contain the given label.
std::vector<int> ParseIterations(const std::string& text, const std::string& label) {
    std::vector<int> iters;
    size_t pos = 0;
    while ((pos = text.find(label, pos)) != std::string::npos) {
        size_t eol = text.find('\n', pos);
        size_t it = text.find("#Iter=", pos);
        if (it != std::string::npos && it < eol)
            iters.push_back(std::stoi(text.substr(it + 6)));
        pos = (eol == std::string::npos) ? text.size() : eol;
    }
    return iters;
}

State RunCouette(double atol, int check_interval) {
    double density = 0.001;    // g/mm3
    double viscosity = 0.001;  // g/(mm.s2)
    double g = gravity;
    double fluid_height = 2.0;
    double cylinder_height = 1.2 * fluid_height;

    ChSystemNSC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetVerbose(true);  // for the iteration counts; verbose output only prints, it does not change the computation

    const ChVector3d gravity(0, -g, 0);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = density;
    fluid_props.viscosity = viscosity;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::IMPLICIT_SPH;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 1;
    sph_params.shifting_xsph_eps = 0.1;
    sph_params.shifting_beta_implicit = 0.0;
    sph_params.min_distance_coefficient = 0.001;
    sph_params.use_density_based_projection = true;
    sph_params.num_proximity_search_steps = 1;
    sysSPH.SetSPHParameters(sph_params);

    ChFsiFluidSystemSPH::LinSolverParameters linsolv_params;
    linsolv_params.type = SolverType::JACOBI;
    linsolv_params.atol = atol;
    linsolv_params.rtol = 0;
    linsolv_params.max_num_iters = max_num_iters;
    linsolv_params.check_interval = check_interval;
    sysSPH.SetLinSolverParameters(linsolv_params);

    double bxDim = 14.0, byDim = 10.0, bzDim = 14.0;
    sysSPH.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim * 1.2, -bzDim / 2 * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim * 1.2, bzDim / 2 * 1.2);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_ALL_PERIODIC);

    // Fluid annulus (cylinder axis along Y), hydrostatic initial pressure
    auto points = sysSPH.CreatePointsCylinderAnnulus(inner_radius + initial_spacing / 2, outer_radius - initial_spacing / 2, fluid_height, true);
    for (const auto& p : points) {
        double x = p.x();
        double y = p.z();
        double z = -p.y();
        double pressure = g * density * (fluid_height - y);
        sysSPH.AddSPHParticle({x, y, z}, density, pressure, viscosity);
    }

    // Fixed bottom plate
    auto bottom_plate = chrono_types::make_shared<ChBody>();
    bottom_plate->SetPos(ChVector3d(0, -fluid_height / 2 - initial_spacing, 0));
    bottom_plate->SetFixed(true);
    sysMBS.AddBody(bottom_plate);
    auto bce_plate = sysSPH.CreatePointsPlate(ChVector2d(outer_radius * 2.5, outer_radius * 2.5));
    sysFSI.AddFsiBoundary(bce_plate, ChFrame<>(ChVector3d(0, -fluid_height / 2 - initial_spacing, 0), Q_ROTATE_Z_TO_Y));

    // Fixed inner cylinder, outer cylinder driven by a motor
    ChVector3d cylinder_center(0, cylinder_height / 2 - fluid_height / 2, 0);

    auto inner_cylinder = chrono_types::make_shared<ChBody>();
    inner_cylinder->SetFixed(true);
    inner_cylinder->EnableCollision(false);
    sysMBS.AddBody(inner_cylinder);
    auto bce_inner = sysSPH.CreatePointsCylinderAnnulus(inner_radius - 3 * initial_spacing, inner_radius - initial_spacing / 2, cylinder_height, true);
    sysFSI.AddRigidBody(inner_cylinder, bce_inner, ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y), false);

    auto outer_cylinder = chrono_types::make_shared<ChBody>();
    outer_cylinder->SetMass(1.0);
    outer_cylinder->SetInertia(ChMatrix33d(ChVector3d(1, 1, 1)));
    outer_cylinder->EnableCollision(false);
    sysMBS.AddBody(outer_cylinder);
    auto bce_outer = sysSPH.CreatePointsCylinderAnnulus(outer_radius + initial_spacing / 2, outer_radius + 3 * initial_spacing, cylinder_height, true);
    sysFSI.AddRigidBody(outer_cylinder, bce_outer, ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y), false);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(omega));
    motor->Initialize(outer_cylinder, bottom_plate, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));
    sysMBS.AddLink(motor);

    // The solver components latch the verbose flag in Initialize, so capture from there on
    StdoutCapture capture;
    sysFSI.Initialize();
    for (int step = 0; step < num_steps; step++)
        sysFSI.DoStepDynamics(step_size);
    std::string log = capture.Finish();

    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();
    auto props = sysSPH.GetParticleFluidProperties();
    size_t n = std::min(pos.size(), sysSPH.GetNumFluidMarkers() + sysSPH.GetNumBoundaryMarkers());

    State state;
    state.finite = n > 0;
    for (size_t i = 0; i < n; i++) {
        state.records.push_back({pos[i].x(), pos[i].y(), pos[i].z(), vel[i].x(), vel[i].y(), vel[i].z(), props[i].x(), props[i].y(), props[i].z()});
        state.finite = state.finite && std::isfinite(pos[i].Length()) && std::isfinite(vel[i].Length()) && std::isfinite(props[i].Length());
        state.vmax = std::max(state.vmax, vel[i].Length());
    }
    std::sort(state.records.begin(), state.records.end());
    state.vstar_iters = ParseIterations(log, "V_star_Predictor Equation");
    state.pressure_iters = ParseIterations(log, "Pressure Poisson Equation");
    return state;
}

int main(int argc, char* argv[]) {
    // Pressure tolerances: none (fixed iteration count), huge (stop after 3 iterations), intermediate
    const std::vector<double> tolerances = {0.0, 1e30, 1e-7};
    const std::vector<int> intervals = {1, 7, 10};

    std::vector<State> reference;
    for (double atol : tolerances) {
        cout << "atol = " << atol << endl;
        State ref = RunCouette(atol, intervals[0]);
        for (size_t i = 1; i < intervals.size(); i++) {
            State other = RunCouette(atol, intervals[i]);
            check(other == ref, "check_interval " + std::to_string(intervals[i]) + " matches check_interval 1 bitwise");
            check(other.vstar_iters == ref.vstar_iters && other.pressure_iters == ref.pressure_iters,
                  "check_interval " + std::to_string(intervals[i]) + " gives the same iteration counts as check_interval 1");
        }

        auto& vi = ref.vstar_iters;
        auto& pi = ref.pressure_iters;
        cout << "  pressure iterations:";
        for (int k : pi)
            cout << " " << k;
        cout << endl;
        check(vi.size() == (size_t)num_steps && pi.size() == (size_t)num_steps, "one V* and one pressure solve reported per step");
        check(std::all_of(vi.begin(), vi.end(), [](int k) { return k >= 3 && k <= max_num_iters; }), "V* iterations within [3, max_num_iters]");
        if (atol == 0) {
            check(!pi.empty() && std::all_of(pi.begin(), pi.end(), [](int k) { return k == max_num_iters; }), "every pressure solve runs max_num_iters iterations");
        } else if (atol > 1) {
            check(!pi.empty() && std::all_of(pi.begin(), pi.end(), [](int k) { return k == 3; }), "every pressure solve stops after exactly 3 iterations");
        } else {
            check(std::all_of(pi.begin(), pi.end(), [](int k) { return k >= 3 && k <= max_num_iters; }), "pressure iterations within [3, max_num_iters]");
            check(std::any_of(pi.begin(), pi.end(), [&](int k) { return k < max_num_iters && k % intervals[1] != 0 && k % intervals[2] != 0; }),
                  "some pressure solve stops early between two host checks");
        }

        double vbound = 2 * gravity * num_steps * step_size + omega * outer_radius;
        check(ref.finite, "particle state is finite");
        check(ref.vmax < vbound, "max speed " + std::to_string(ref.vmax) + " below " + std::to_string(vbound));

        reference.push_back(std::move(ref));
    }

    check(!(reference[1] == reference[0]), "huge atol changes the result (early exit exercised)");
    check(!(reference[2] == reference[0]) && !(reference[2] == reference[1]), "intermediate atol gives a third result");

    if (num_failures) {
        cout << "\n" << num_failures << " check(s) failed" << endl;
        return 1;
    }
    cout << "\nAll checks passed" << endl;
    return 0;
}
