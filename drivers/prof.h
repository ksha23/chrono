// Headless profiling hooks for the multicore-peri-modal campaign (not part of Chrono).
// Env: PROF_THREADS (override thread count), PROF_WARM (steps excluded), PROF_STEPS (timed steps, then exit),
//      PROF_TAG (label printed in the report line).
#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include "chrono/physics/ChSystem.h"
#ifdef PROF_MULTICORE
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"
#endif

namespace prof {
inline int env_int(const char* n, int d) {
    const char* v = std::getenv(n);
    return v ? std::atoi(v) : d;
}
inline int threads(int d) { return env_int("PROF_THREADS", d); }

struct State {
    int step = 0, warm = 0, nsteps = 0;
    bool registered = false, done = false;
    std::vector<double> wall;
    std::map<std::string, double> acc;
    double contacts = 0, iters = 0, bodies = 0;
    double ck = 0;  // state hash, refreshed after every step (report() may run from atexit after the system is gone)
    chrono::ChSystem* sys = nullptr;
};
inline State& S() {
    static State s;
    return s;
}
inline void report() {
    State& s = S();
    if (s.done)
        return;
    s.done = true;
    int n = (int)s.wall.size();
    if (n == 0) {
        std::printf("PROF_RESULT tag=%s timed_steps=0\n", std::getenv("PROF_TAG") ? std::getenv("PROF_TAG") : "");
        std::fflush(stdout);
        return;
    }
    std::vector<double> w = s.wall;
    std::sort(w.begin(), w.end());
    double sum = 0;
    for (double x : w)
        sum += x;
    std::printf("PROF_RESULT tag=%s threads=%d timed_steps=%d warm=%d mean_ms=%.4f median_ms=%.4f min_ms=%.4f max_ms=%.4f",
                std::getenv("PROF_TAG") ? std::getenv("PROF_TAG") : "", env_int("PROF_THREADS", -1), n, s.warm,
                1e3 * sum / n, 1e3 * w[n / 2], 1e3 * w[0], 1e3 * w[n - 1]);
    for (auto& kv : s.acc)
        std::printf(" %s_ms=%.4f", kv.first.c_str(), 1e3 * kv.second / n);
    double ck = s.ck;
    std::printf(" bodies=%.0f contacts=%.1f iters=%.2f checksum=%.17g\n", s.bodies / n, s.contacts / n, s.iters / n, ck);
    std::fflush(stdout);
}
inline void after(chrono::ChSystem& sys, double dt_wall) {
    State& s = S();
    s.sys = &sys;
    if (s.step >= s.warm && !s.done) {
        s.wall.push_back(dt_wall);
        s.acc["t_step"] += sys.GetTimerStep();
        s.acc["t_coll"] += sys.GetTimerCollision();
        s.acc["t_broad"] += sys.GetTimerCollisionBroad();
        s.acc["t_narrow"] += sys.GetTimerCollisionNarrow();
        s.acc["t_update"] += sys.GetTimerUpdate();
        s.acc["t_advance"] += sys.GetTimerAdvance();
        s.acc["t_lssetup"] += sys.GetTimerLSsetup();
        s.acc["t_lssolve"] += sys.GetTimerLSsolve();
        s.acc["t_jac"] += sys.GetTimerJacobian();
        s.acc["t_setup"] += sys.GetTimerSetup();
        s.contacts += sys.GetNumContacts();
        s.bodies += sys.GetNumBodiesActive();
#ifdef PROF_MULTICORE
        if (auto* mc = dynamic_cast<chrono::ChSystemMulticore*>(&sys)) {
            auto& T = mc->data_manager->system_timer;
            s.acc["mc_stab"] += T.GetTime("ChIterativeSolverMulticore_Stab");
            s.acc["mc_smc_process"] += T.GetTime("ChIterativeSolverMulticoreSMC_ProcessContact");
            s.acc["mc_schur"] += T.GetTime("SchurProduct");
            s.acc["mc_project"] += T.GetTime("ChSolverMulticore_Project");
            s.acc["mc_D"] += T.GetTime("ChIterativeSolverMulticore_D");
            s.acc["mc_E"] += T.GetTime("ChIterativeSolverMulticore_E");
            s.acc["mc_R"] += T.GetTime("ChIterativeSolverMulticore_R");
            s.acc["mc_N"] += T.GetTime("ChIterativeSolverMulticore_N");
            s.acc["mc_solve"] += T.GetTime("ChSolverMulticore_Solve");
            if (auto it = std::dynamic_pointer_cast<chrono::ChIterativeSolverMulticore>(sys.GetSolver()))
                s.iters += it->GetIterations();
        }
#endif
    }
    s.ck = 0;
    for (auto& b : sys.GetBodies())
        s.ck += b->GetPos().x() + 2 * b->GetPos().y() + 3 * b->GetPos().z() + 1e-3 * b->GetPosDt().Length();
    s.step++;
    if (s.step >= s.warm + s.nsteps) {
        report();
        std::_Exit(0);
    }
}
inline void step(chrono::ChSystem& sys, double dt) {
    State& s = S();
    if (!s.registered) {
        s.registered = true;
        s.warm = env_int("PROF_WARM", 0);
        s.nsteps = env_int("PROF_STEPS", 1000);
        std::atexit(report);
    }
    auto t0 = std::chrono::steady_clock::now();
    sys.DoStepDynamics(dt);
    auto t1 = std::chrono::steady_clock::now();
    after(sys, std::chrono::duration<double>(t1 - t0).count());
}
inline void step(chrono::ChSystem* sys, double dt) { step(*sys, dt); }
}  // namespace prof
