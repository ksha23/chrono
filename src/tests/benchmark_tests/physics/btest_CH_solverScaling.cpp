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
// Core-level solver survey harness.
//
// Three studies, none of which involve Chrono::Vehicle:
//
//   --case=chain      articulated chain of N bodies, bilateral constraints only.
//                     Sweeps N to locate the size at which a sparse direct
//                     solver stops being cheaper than a fixed-budget iterative
//                     solver.
//
//   --case=massratio  two-body system whose mass ratio is swept over several
//                     decades, to quantify how Schur-complement conditioning
//                     degrades projected-gradient accuracy.
//
//   --case=granular   NSC spheres in a container: a genuinely unilateral,
//                     contact-dominated problem where a direct solver is not
//                     applicable.  Used to check that the automatic solver
//                     selection does NOT take the direct path.
//
// This is a measurement tool, not a pass/fail test.
//
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

using namespace chrono;

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
// Model builders
// =============================================================================

// Chain of N boxes connected by revolute joints, anchored to ground.
// Purely bilateral: no contacts, no unilateral constraints.
static void BuildChain(ChSystemNSC& sys, int n) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    std::shared_ptr<ChBody> prev = ground;
    for (int i = 0; i < n; i++) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetMass(1.0);
        body->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
        body->SetPos(ChVector3d(0.5 + i * 1.0, 0, 0));
        sys.AddBody(body);

        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(prev, body, ChFrame<>(ChVector3d(i * 1.0, 0, 0), QUNIT));
        sys.AddLink(rev);

        prev = body;
    }
}

// Heavy body suspended from ground by a revolute joint, carrying a light body.
// Mass ratio between the two is the swept parameter.
static void BuildMassRatio(ChSystemNSC& sys, double heavy, double light) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto big = chrono_types::make_shared<ChBody>();
    big->SetMass(heavy);
    big->SetInertiaXX(ChVector3d(heavy, heavy, heavy));
    big->SetPos(ChVector3d(1, 0, 0));
    sys.AddBody(big);

    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    rev->Initialize(ground, big, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sys.AddLink(rev);

    // A short chain of light bodies hanging off the heavy one.
    std::shared_ptr<ChBody> prev = big;
    for (int i = 0; i < 4; i++) {
        auto small = chrono_types::make_shared<ChBody>();
        small->SetMass(light);
        small->SetInertiaXX(ChVector3d(light * 0.01, light * 0.01, light * 0.01));
        small->SetPos(ChVector3d(2.0 + i, 0, 0));
        sys.AddBody(small);

        auto r2 = chrono_types::make_shared<ChLinkLockRevolute>();
        r2->Initialize(prev, small, ChFrame<>(ChVector3d(1.5 + i, 0, 0), QUNIT));
        sys.AddLink(r2);
        prev = small;
    }
}

// A 3-D lattice of bodies tied to their neighbours by distance constraints.
// Bilateral like the chain, but with 3-D connectivity, so a sparse factorization
// suffers far more fill-in than in the (banded) chain case.  This is the
// topology that decides where a direct solver stops being the cheaper option.
static void BuildLattice(ChSystemNSC& sys, int k) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    std::vector<std::vector<std::vector<std::shared_ptr<ChBody>>>> grid(
        k, std::vector<std::vector<std::shared_ptr<ChBody>>>(k, std::vector<std::shared_ptr<ChBody>>(k)));

    for (int i = 0; i < k; i++)
        for (int j = 0; j < k; j++)
            for (int l = 0; l < k; l++) {
                auto b = chrono_types::make_shared<ChBody>();
                b->SetMass(1.0);
                b->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
                b->SetPos(ChVector3d(i, j, l));
                sys.AddBody(b);
                grid[i][j][l] = b;
            }

    auto tie = [&](std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b) {
        auto d = chrono_types::make_shared<ChLinkDistance>();
        d->Initialize(a, b, false, a->GetPos(), b->GetPos());
        sys.AddLink(d);
    };

    for (int i = 0; i < k; i++)
        for (int j = 0; j < k; j++)
            for (int l = 0; l < k; l++) {
                if (i + 1 < k)
                    tie(grid[i][j][l], grid[i + 1][j][l]);
                if (j + 1 < k)
                    tie(grid[i][j][l], grid[i][j + 1][l]);
                if (l + 1 < k)
                    tie(grid[i][j][l], grid[i][j][l + 1]);
            }

    // Anchor one corner so the lattice is not free-floating.
    auto fix = chrono_types::make_shared<ChLinkLockLock>();
    fix->Initialize(ground, grid[0][0][0], ChFrame<>(grid[0][0][0]->GetPos(), QUNIT));
    sys.AddLink(fix);
}

// Spheres poured into a box container: unilateral, contact dominated.
static void BuildGranular(ChSystemNSC& sys, int nx, int ny, int nz) {
    // The collision system must be selected explicitly, otherwise no contact is ever
    // generated and the case silently degenerates into a contact-free model.
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    double radius = 0.05;
    double spacing = 2.2 * radius;

    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    container->EnableCollision(true);
    double hx = nx * spacing * 0.6;
    double hy = ny * spacing * 0.6;
    utils::AddBoxGeometry(container.get(), mat, ChVector3d(2 * hx, 2 * hy, 0.1), ChVector3d(0, 0, -0.05));
    utils::AddBoxGeometry(container.get(), mat, ChVector3d(0.1, 2 * hy, 1.0), ChVector3d(-hx, 0, 0.5));
    utils::AddBoxGeometry(container.get(), mat, ChVector3d(0.1, 2 * hy, 1.0), ChVector3d(hx, 0, 0.5));
    utils::AddBoxGeometry(container.get(), mat, ChVector3d(2 * hx, 0.1, 1.0), ChVector3d(0, -hy, 0.5));
    utils::AddBoxGeometry(container.get(), mat, ChVector3d(2 * hx, 0.1, 1.0), ChVector3d(0, hy, 0.5));
    sys.AddBody(container);

    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            for (int iz = 0; iz < nz; iz++) {
                auto ball = chrono_types::make_shared<ChBodyEasySphere>(radius, 1000, true, true, mat);
                ball->SetPos(ChVector3d((ix - nx / 2.0) * spacing, (iy - ny / 2.0) * spacing, 0.1 + iz * spacing));
                sys.AddBody(ball);
            }
        }
    }
}

// =============================================================================

struct Sample {
    double us_step = 0;
    double us_ls_solve = 0;
    double us_ls_setup = 0;
    unsigned int nvars = 0;
    unsigned int nconstr = 0;
    unsigned int nunilat = 0;
    unsigned int ncontact = 0;
    double fill_ratio = 0;  // nnz(L+U)/n for direct solvers
    long armijo = 0;        // total Armijo backtracks (BB only)
    double dev = 0;     // vs reference, if provided
    double mean_it = 0;
    double early = 0;
};

// Configure the solver on a system.
static void ApplySolver(ChSystem& sys, const std::string& name, int iters, double tol, int warm) {
    if (name == "SPARSE_LU") {
        sys.SetSolverType(ChSolver::Type::SPARSE_LU);
        sys.GetSolver()->AsDirect()->LockSparsityPattern(true);
        return;
    }
    if (name == "SPARSE_QR") {
        sys.SetSolverType(ChSolver::Type::SPARSE_QR);
        sys.GetSolver()->AsDirect()->LockSparsityPattern(true);
        return;
    }
    if (name == "BB")
        sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    else if (name == "PSOR")
        sys.SetSolverType(ChSolver::Type::PSOR);
    else if (name == "APGD")
        sys.SetSolverType(ChSolver::Type::APGD);
    else if (name == "MINRES")
        sys.SetSolverType(ChSolver::Type::MINRES);
    else if (name == "AUTO")
        sys.SetSolverType(ChSolver::Type::AUTO);
    else if (name == "DEFAULT")
        return;  // whatever ChSystemNSC installs
    else {
        std::cerr << "unknown solver " << name << std::endl;
        std::exit(1);
    }

    auto it = sys.GetSolver()->AsIterative();
    if (it) {
        if (iters > 0)
            it->SetMaxIterations(iters);
        if (tol >= 0)
            it->SetTolerance(tol);
        if (warm >= 0)
            it->EnableWarmStart(warm != 0);
    }
}

// Run a system for n_steps, timing the last (n_steps - warmup).
static Sample Run(ChSystem& sys, int n_steps, int warmup, double step, std::vector<ChVector3d>* traj) {
    Sample s;
    auto t0 = std::chrono::steady_clock::now();
    double ls_solve0 = 0, ls_setup0 = 0;
    double sum_it = 0;
    int n_it = 0, n_early = 0;
    auto solver = sys.GetSolver();

    for (int i = 0; i < n_steps; i++) {
        sys.DoStepDynamics(step);
        if (i == warmup) {
            t0 = std::chrono::steady_clock::now();
            ls_solve0 = sys.GetTimerLSsolve();
            ls_setup0 = sys.GetTimerLSsetup();
        }
        if (traj) {
            // Track the last non-fixed body, the most constraint-sensitive point.
            for (auto it = sys.GetBodies().rbegin(); it != sys.GetBodies().rend(); ++it) {
                if (!(*it)->IsFixed()) {
                    traj->push_back((*it)->GetPos());
                    break;
                }
            }
        }
        if (solver->IsIterative()) {
            auto is = solver->AsIterative();
            sum_it += is->GetIterations();
            n_it++;
            if (is->GetIterations() < is->GetMaxIterations())
                n_early++;
        }
        if (auto bb = std::dynamic_pointer_cast<ChSolverBB>(sys.GetSolver()))
            s.armijo += bb->GetNumArmijoBacktracks();
        s.ncontact = std::max(s.ncontact, sys.GetNumContacts());
    }
    auto t1 = std::chrono::steady_clock::now();
    int n = n_steps - warmup;
    s.us_step = std::chrono::duration<double, std::micro>(t1 - t0).count() / n;
    s.us_ls_solve = (sys.GetTimerLSsolve() - ls_solve0) * 1e6 / n;
    s.us_ls_setup = (sys.GetTimerLSsetup() - ls_setup0) * 1e6 / n;
    s.mean_it = n_it ? sum_it / n_it : 0;
    s.early = n_it ? double(n_early) / n_it : 0;

    auto d = sys.GetSystemDescriptor();
    s.nvars = d->CountActiveVariables();
    s.nconstr = d->CountActiveConstraints();
    if (solver->IsDirect())
        s.fill_ratio = double(solver->AsDirect()->GetNumFactorNonzeros()) / double(s.nvars + s.nconstr);
    for (auto c : d->GetConstraints())
        if (c->IsActive() && c->IsUnilateral())
            s.nunilat++;
    return s;
}

static double MaxDev(const std::vector<ChVector3d>& a, const std::vector<ChVector3d>& b) {
    double m = 0;
    size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; i++) {
        double d = (a[i] - b[i]).Length();
        if (!std::isfinite(d))
            return std::numeric_limits<double>::infinity();
        m = std::max(m, d);
    }
    return m;
}

// =============================================================================

int main(int argc, char** argv) {
    Args args(argc, argv);
    std::string which = args.Get("case", "chain");
    int n_steps = args.GetInt("steps", 600);
    int warmup = args.GetInt("warmup", 100);
    double step = args.GetDouble("step", 2e-3);
    int iters = args.GetInt("iters", 150);
    double tol = args.GetDouble("tol", -1);
    int warm = args.GetInt("warm", -1);

    std::cout << std::setprecision(6);

    if (which == "chain") {
        std::cout << "case,n_bodies,solver,nvars,nconstr,us_step,us_ls_solve,us_ls_setup,max_dev,mean_it,early\n";
        std::vector<int> sizes;
        if (args.Has("n"))
            sizes.push_back(args.GetInt("n", 10));
        else
            sizes = {5, 10, 20, 40, 80, 160, 320, 640, 1280};

        for (int n : sizes) {
            // Reference: sparse LU
            std::vector<ChVector3d> ref;
            {
                ChSystemNSC sys;
                BuildChain(sys, n);
                ApplySolver(sys, "SPARSE_LU", 0, -1, -1);
                auto s = Run(sys, n_steps, warmup, step, &ref);
                std::cout << "chain," << n << ",SPARSE_LU," << s.nvars << "," << s.nconstr << "," << s.us_step << ","
                          << s.us_ls_solve << "," << s.us_ls_setup << ",0,0,0,fill=" << s.fill_ratio << "\n";
            }
            std::vector<std::string> svs = {"SPARSE_QR", "BB", "PSOR", "APGD", "MINRES"};
            if (args.Has("solvers")) {
                svs.clear();
                std::string spec = args.Get("solvers", "");
                size_t p = 0;
                while (p <= spec.size()) {
                    size_t q = spec.find('+', p);
                    if (q == std::string::npos)
                        q = spec.size();
                    if (q > p)
                        svs.push_back(spec.substr(p, q - p));
                    p = q + 1;
                }
            }
            for (std::string sv : svs) {
                std::vector<ChVector3d> traj;
                ChSystemNSC sys;
                BuildChain(sys, n);
                ApplySolver(sys, sv, iters, tol, warm);
                auto s = Run(sys, n_steps, warmup, step, &traj);
                std::cout << "chain," << n << "," << sv << "," << s.nvars << "," << s.nconstr << "," << s.us_step
                          << "," << s.us_ls_solve << "," << s.us_ls_setup << "," << MaxDev(ref, traj) << ","
                          << s.mean_it << "," << s.early << ",armijo=" << s.armijo << "\n";
            }
        }
        return 0;
    }

    if (which == "lattice") {
        std::cout << "case,k,n_bodies,solver,nvars,nconstr,us_step,us_ls_solve,us_ls_setup,max_dev,mean_it,early\n";
        std::vector<int> sizes;
        if (args.Has("k"))
            sizes.push_back(args.GetInt("k", 4));
        else
            sizes = {3, 4, 5, 6, 8, 10, 12};

        for (int k : sizes) {
            std::vector<ChVector3d> ref;
            {
                ChSystemNSC sys;
                BuildLattice(sys, k);
                ApplySolver(sys, "SPARSE_LU", 0, -1, -1);
                auto s = Run(sys, n_steps, warmup, step, &ref);
                std::cout << "lattice," << k << "," << k * k * k << ",SPARSE_LU," << s.nvars << "," << s.nconstr << ","
                          << s.us_step << "," << s.us_ls_solve << "," << s.us_ls_setup << ",0,0,0,fill=" << s.fill_ratio << "\n";
            }
            for (std::string sv : {std::string("BB"), std::string("MINRES")}) {
                std::vector<ChVector3d> traj;
                ChSystemNSC sys;
                BuildLattice(sys, k);
                ApplySolver(sys, sv, iters, tol, warm);
                auto s = Run(sys, n_steps, warmup, step, &traj);
                std::cout << "lattice," << k << "," << k * k * k << "," << sv << "," << s.nvars << "," << s.nconstr
                          << "," << s.us_step << "," << s.us_ls_solve << "," << s.us_ls_setup << ","
                          << MaxDev(ref, traj) << "," << s.mean_it << "," << s.early << "\n";
            }
        }
        return 0;
    }

    if (which == "massratio") {
        std::cout << "case,mass_ratio,solver,iters,nconstr,max_dev,mean_it,early,final_err\n";
        double light = 5.0;
        for (double ratio : {1.0, 3.0, 10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0}) {
            std::vector<ChVector3d> ref;
            {
                ChSystemNSC sys;
                BuildMassRatio(sys, light * ratio, light);
                ApplySolver(sys, "SPARSE_LU", 0, -1, -1);
                Run(sys, n_steps, warmup, step, &ref);
            }
            for (int it : {50, 150, 500, 2000}) {
                std::vector<ChVector3d> traj;
                ChSystemNSC sys;
                BuildMassRatio(sys, light * ratio, light);
                ApplySolver(sys, "BB", it, tol, warm);
                auto s = Run(sys, n_steps, warmup, step, &traj);
                std::cout << "massratio," << ratio << ",BB," << it << "," << s.nconstr << "," << MaxDev(ref, traj)
                          << "," << s.mean_it << "," << s.early << ","
                          << sys.GetSolver()->AsIterative()->GetError() << "\n";
            }
        }
        return 0;
    }

    if (which == "granular") {
        std::cout << "case,n_spheres,solver,nvars,nconstr,nunilat,ncontact,us_step,mean_it,early\n";
        int nx = args.GetInt("nx", 10);
        int ny = args.GetInt("ny", 10);
        int nz = args.GetInt("nz", 6);
        std::string sv = args.Get("solver", "DEFAULT");
        ChSystemNSC sys;
        BuildGranular(sys, nx, ny, nz);
        ApplySolver(sys, sv, iters, tol, warm);
        auto s = Run(sys, n_steps, warmup, step, nullptr);
        std::cout << "granular," << nx * ny * nz << "," << sys.GetSolver()->GetTypeAsString() << "," << s.nvars << ","
                  << s.nconstr << "," << s.nunilat << "," << s.ncontact << "," << s.us_step << "," << s.mean_it << ","
                  << s.early << "\n";
        return 0;
    }

    std::cerr << "unknown case " << which << std::endl;
    return 1;
}
