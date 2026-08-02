// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Chrono solver survey
// =============================================================================
//
// Solver survey harness: HMMWV double lane change.
//
// Runs the same maneuver with a configurable solver and reports cost and the
// deviation of the resulting chassis trajectory from a reference trajectory
// (normally produced with a sparse direct solver, which solves the index-2 DAE
// to machine precision).
//
// This is a measurement tool, not a pass/fail test.  Usage:
//
//   btest_VEH_solverSurvey --solver=SPARSE_LU --lock=1 --ref=traj_ref.txt
//   btest_VEH_solverSurvey --solver=BB --iters=150 --refin=traj_ref.txt
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChSolverADMM.h"
#include "chrono/solver/ChSolverAuto.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Command line parsing (minimal, no dependency on the demo utils)
// =============================================================================

class Args {
  public:
    Args(int argc, char** argv) {
        for (int i = 1; i < argc; i++) {
            std::string a = argv[i];
            if (a.rfind("--", 0) != 0)
                continue;
            a = a.substr(2);
            auto pos = a.find('=');
            if (pos == std::string::npos)
                m_map[a] = "1";
            else
                m_map[a.substr(0, pos)] = a.substr(pos + 1);
        }
    }
    bool Has(const std::string& k) const { return m_map.count(k) > 0; }
    std::string Get(const std::string& k, const std::string& def) const {
        auto it = m_map.find(k);
        return it == m_map.end() ? def : it->second;
    }
    int GetInt(const std::string& k, int def) const {
        auto it = m_map.find(k);
        return it == m_map.end() ? def : std::stoi(it->second);
    }
    double GetDouble(const std::string& k, double def) const {
        auto it = m_map.find(k);
        return it == m_map.end() ? def : std::stod(it->second);
    }

  private:
    std::map<std::string, std::string> m_map;
};

// =============================================================================

static ChSolver::Type ParseSolver(const std::string& s) {
    if (s == "PSOR")
        return ChSolver::Type::PSOR;
    if (s == "PJACOBI")
        return ChSolver::Type::PJACOBI;
    if (s == "BB" || s == "BARZILAIBORWEIN")
        return ChSolver::Type::BARZILAIBORWEIN;
    if (s == "APGD")
        return ChSolver::Type::APGD;
    if (s == "ADMM")
        return ChSolver::Type::ADMM;
    if (s == "MINRES")
        return ChSolver::Type::MINRES;
    if (s == "GMRES")
        return ChSolver::Type::GMRES;
    if (s == "BICGSTAB")
        return ChSolver::Type::BICGSTAB;
    if (s == "SPARSE_LU")
        return ChSolver::Type::SPARSE_LU;
    if (s == "SPARSE_QR")
        return ChSolver::Type::SPARSE_QR;
    if (s == "AUTO")
        return ChSolver::Type::AUTO;
    if (s == "DEFAULT")
        return ChSolver::Type::CUSTOM;  // leave whatever ChVehicle set
    std::cerr << "Unknown solver '" << s << "'" << std::endl;
    std::exit(1);
}

static TireModelType ParseTire(const std::string& s) {
    if (s == "TMEASY")
        return TireModelType::TMEASY;
    if (s == "FIALA")
        return TireModelType::FIALA;
    if (s == "RIGID")
        return TireModelType::RIGID;
    if (s == "RIGID_MESH")
        return TireModelType::RIGID_MESH;
    if (s == "PAC02")
        return TireModelType::PAC02;
    std::cerr << "Unknown tire '" << s << "'" << std::endl;
    std::exit(1);
}

// =============================================================================

struct Result {
    double us_per_step_total = 0;
    double us_per_step_ls = 0;      // solver Solve() only
    double us_per_step_setup = 0;   // solver Setup() only
    double max_dev = 0;
    double rms_dev = 0;
    double mean_iters = 0;
    double early_exit_frac = 0;
    double final_err = 0;
    unsigned int n_vars = 0;
    unsigned int n_constr = 0;
    unsigned int n_unilateral = 0;
    unsigned int n_contacts_max = 0;
    bool diverged = false;
};

int main(int argc, char** argv) {
    Args args(argc, argv);

    if (args.Has("help")) {
        std::cout << "btest_VEH_solverSurvey options:\n"
                  << "  --solver=PSOR|PJACOBI|BB|APGD|ADMM|MINRES|GMRES|BICGSTAB|SPARSE_LU|SPARSE_QR|DEFAULT\n"
                  << "  --iters=N            max iterations (iterative solvers)\n"
                  << "  --tol=X              tolerance      (iterative solvers)\n"
                  << "  --warm=0|1           warm start\n"
                  << "  --lock=0|1           lock sparsity pattern (direct solvers)\n"
                  << "  --tire=TMEASY|FIALA|RIGID|RIGID_MESH|PAC02\n"
                  << "  --contact=NSC|SMC\n"
                  << "  --steps=N            total simulated steps (default 7500)\n"
                  << "  --warmup=N           steps excluded from timing (default 2500)\n"
                  << "  --ref=FILE           write this run's trajectory as the reference\n"
                  << "  --refin=FILE         compare against this reference trajectory\n"
                  << "  --label=NAME         label used in the CSV output row\n"
                  << "  --header             emit the CSV header line and exit\n";
        return 0;
    }

    if (args.Has("header")) {
        std::cout << "label,solver,iters,tol,warm,lock,tire,contact,us_step,us_ls_solve,us_ls_setup,"
                  << "max_dev,rms_dev,mean_iters,early_exit_frac,final_err,nvars,nconstr,nunilat,ncontact\n";
        return 0;
    }

    std::string solver_name = args.Get("solver", "DEFAULT");
    std::string tire_name = args.Get("tire", "TMEASY");
    std::string contact_name = args.Get("contact", "SMC");
    int max_iters = args.GetInt("iters", -1);
    double tol = args.GetDouble("tol", -1);
    int warm = args.GetInt("warm", -1);
    int lock = args.GetInt("lock", 1);
    int n_steps = args.GetInt("steps", 7500);
    int n_warmup = args.GetInt("warmup", 2500);
    std::string label = args.Get("label", solver_name);

    const double step_veh = 2e-3;
    const double step_tire = 1e-3;

    // ---------------------------------------------------------------- model
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(contact_name == "NSC" ? ChContactMethod::NSC : ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-120, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(ParseTire(tire_name));
    hmmwv.SetTireStepSize(step_tire);
    hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::NONE);
    hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv.SetTireVisualizationType(VisualizationType::NONE);

    ChSystem* sys = hmmwv.GetSystem();

    // --------------------------------------------------------------- solver
    ChSolver::Type stype = ParseSolver(solver_name);
    if (stype != ChSolver::Type::CUSTOM)
        sys->SetSolverType(stype);

    auto solver = sys->GetSolver();
    if (solver->IsIterative()) {
        auto it = solver->AsIterative();
        if (max_iters > 0)
            it->SetMaxIterations(max_iters);
        if (tol >= 0)
            it->SetTolerance(tol);
        if (warm >= 0)
            it->EnableWarmStart(warm != 0);
    }
    if (solver->IsDirect()) {
        solver->AsDirect()->LockSparsityPattern(lock != 0);
    }

    // -------------------------------------------------------------- terrain
    RigidTerrain terrain(sys);
    // The patch material must match the system's contact method, otherwise the
    // terrain generates no contacts at all.
    std::shared_ptr<ChContactMaterial> patch_material;
    if (contact_name == "NSC") {
        auto m = chrono_types::make_shared<ChContactMaterialNSC>();
        m->SetFriction(0.9f);
        m->SetRestitution(0.01f);
        patch_material = m;
    } else {
        auto m = chrono_types::make_shared<ChContactMaterialSMC>();
        m->SetFriction(0.9f);
        m->SetRestitution(0.01f);
        m->SetYoungModulus(2e7f);
        patch_material = m;
    }
    auto patch = terrain.AddPatch(patch_material, CSYSNORM, 300, 20);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    terrain.Initialize();

    // --------------------------------------------------------------- driver
    auto path = DoubleLaneChangePath(ChVector3d(-125, 0, 0.1), 28.93, 3.6105, 25.0, 50.0, false);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", 12.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ------------------------------------------------------------------ run
    Result res;
    std::vector<ChVector3d> traj;
    traj.reserve(n_steps);

    double sum_iters = 0;
    int n_iter_samples = 0;
    int n_early = 0;

    auto t_start = std::chrono::steady_clock::now();
    auto t_timed = t_start;
    double ls_solve_at_warmup = 0;
    double ls_setup_at_warmup = 0;

    // Open-loop mode replaces the path-follower with a prescribed input profile.  The
    // closed-loop steering/speed controllers are themselves an amplifier of small
    // differences, so the open-loop variant isolates the solver's own contribution.
    bool open_loop = args.Has("openloop");

    for (int i = 0; i < n_steps; i++) {
        double time = sys->GetChTime();

        DriverInputs driver_inputs;
        if (open_loop) {
            driver_inputs.m_throttle = 0.5;
            driver_inputs.m_braking = 0.0;
            // single smooth steer pulse between t = 6 s and t = 9 s
            double u = (time - 6.0) / 3.0;
            driver_inputs.m_steering = (u > 0 && u < 1) ? 0.15 * std::sin(2 * CH_PI * u) : 0.0;
        } else {
            driver_inputs = driver.GetInputs();
            driver.Synchronize(time);
        }
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);

        if (!open_loop)
            driver.Advance(step_veh);
        terrain.Advance(step_veh);
        hmmwv.Advance(step_veh);

        if (i == n_warmup) {
            t_timed = std::chrono::steady_clock::now();
            ls_solve_at_warmup = sys->GetTimerLSsolve();
            ls_setup_at_warmup = sys->GetTimerLSsetup();
        }

        auto pos = hmmwv.GetVehicle().GetPos();
        traj.push_back(pos);
        if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
            res.diverged = true;
            // fill the remainder so trajectories stay comparable in length
            for (int k = i + 1; k < n_steps; k++)
                traj.push_back(pos);
            break;
        }

        // Iteration statistics (iterative solvers only).  Chrono may call Solve()
        // more than once per step; GetIterations() reports the last such call, so
        // this samples the final solve of every step.
        if (solver->IsIterative()) {
            auto it = solver->AsIterative();
            sum_iters += it->GetIterations();
            n_iter_samples++;
            if (it->GetIterations() < it->GetMaxIterations())
                n_early++;
        }
        res.n_contacts_max = std::max(res.n_contacts_max, sys->GetNumContacts());
    }
    auto t_end = std::chrono::steady_clock::now();

    int n_timed = n_steps - n_warmup;
    res.us_per_step_total = std::chrono::duration<double, std::micro>(t_end - t_timed).count() / n_timed;
    res.us_per_step_ls = (sys->GetTimerLSsolve() - ls_solve_at_warmup) * 1e6 / n_timed;
    res.us_per_step_setup = (sys->GetTimerLSsetup() - ls_setup_at_warmup) * 1e6 / n_timed;
    res.mean_iters = n_iter_samples ? sum_iters / n_iter_samples : 0;
    res.early_exit_frac = n_iter_samples ? double(n_early) / n_iter_samples : 0;
    if (solver->IsIterative())
        res.final_err = solver->AsIterative()->GetError();

    // ------------------------------------------------------- problem size
    auto descriptor = sys->GetSystemDescriptor();
    res.n_vars = descriptor->CountActiveVariables();
    res.n_constr = descriptor->CountActiveConstraints();
    for (auto c : descriptor->GetConstraints()) {
        if (c->IsActive() && c->IsUnilateral())
            res.n_unilateral++;
    }

    // ---------------------------------------------------------- trajectory
    if (args.Has("ref")) {
        std::ofstream f(args.Get("ref", "traj_ref.txt"));
        f << std::setprecision(17) << std::scientific;
        for (auto& p : traj)
            f << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    if (args.Has("refin")) {
        std::ifstream f(args.Get("refin", "traj_ref.txt"));
        if (!f) {
            std::cerr << "Cannot open reference trajectory" << std::endl;
            return 1;
        }
        std::ofstream fdev;
        if (args.Has("devout"))
            fdev.open(args.Get("devout", "dev.txt"));

        double x, y, z;
        size_t i = 0;
        double sum2 = 0;
        size_t n = 0;
        while (f >> x >> y >> z && i < traj.size()) {
            double d = (traj[i] - ChVector3d(x, y, z)).Length();
            if (std::isfinite(d)) {
                res.max_dev = std::max(res.max_dev, d);
                sum2 += d * d;
            } else {
                res.max_dev = std::numeric_limits<double>::infinity();
            }
            if (fdev.is_open())
                fdev << i * step_veh << " " << d << "\n";
            n++;
            i++;
        }
        res.rms_dev = n ? std::sqrt(sum2 / n) : 0;
    }

    // ------------------------------------------------------------- report
    std::cout << std::setprecision(6);
    std::string used = solver->GetTypeAsString();
    if (auto sa = std::dynamic_pointer_cast<ChSolverAuto>(solver))
        used += std::string("[") + (sa->UsedDirectSolver() ? "direct" : "iterative") + ":" + sa->WhyIterative() + "]";
    std::cout << label << "," << used << ","
              << (solver->IsIterative() ? solver->AsIterative()->GetMaxIterations() : 0) << ","
              << (solver->IsIterative() ? solver->AsIterative()->GetTolerance() : 0) << "," << warm << "," << lock
              << "," << tire_name << "," << contact_name << "," << res.us_per_step_total << "," << res.us_per_step_ls
              << "," << res.us_per_step_setup << "," << res.max_dev << "," << res.rms_dev << "," << res.mean_iters
              << "," << res.early_exit_frac << "," << res.final_err << "," << res.n_vars << "," << res.n_constr << ","
              << res.n_unilateral << "," << res.n_contacts_max << (res.diverged ? ",DIVERGED" : "") << std::endl;

    return 0;
}
