// Minimal step profiler for headless Chrono::Sensor demo copies (campaign sensor-optix).
// SENPROF_STEPS=N   stop the demo loop after N iterations
// SENPROF_WARMUP=W  exclude the first W iterations (NVRTC/JIT, first BVH build) from stats
#pragma once
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <nvtx3/nvToolsExt.h>

namespace senprof {
using clk = std::chrono::steady_clock;
inline double ms(clk::time_point a, clk::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}
struct Stats {
    long max_steps = -1;
    long warm = 0;
    long steps = 0;
    bool started = false;
    std::vector<double> upd, dyn, other, step;
    clk::time_point t_start, t_last, t_prog;
    double first_update_ms = -1;
    double cur_upd = 0, cur_dyn = 0;
    clk::time_point step_t0;
    Stats() {
        t_prog = clk::now();
        if (const char* s = std::getenv("SENPROF_STEPS")) max_steps = std::atol(s);
        if (const char* s = std::getenv("SENPROF_WARMUP")) warm = std::atol(s);
    }
    static void pct(const char* name, std::vector<double> v) {
        if (v.empty()) return;
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (double x : v) sum += x;
        auto q = [&](double p) { return v[std::min(v.size() - 1, (size_t)(p * (v.size() - 1) + 0.5))]; };
        std::printf("SENPROF %-8s n=%zu mean=%.4f med=%.4f p90=%.4f p99=%.4f max=%.4f sum=%.1f (ms)\n", name, v.size(),
                    sum / v.size(), q(0.5), q(0.9), q(0.99), v.back(), sum);
    }
    void report() {
        static bool done = false;
        if (done) return;
        done = true;
        double total = ms(t_start, t_last);
        long n = (long)step.size();
        std::printf("SENPROF steps=%ld warmup=%ld timed=%ld wall_timed_ms=%.1f per_step_ms=%.4f first_update_ms=%.1f "
                    "setup_to_loop_ms=%.1f\n",
                    steps, warm, n, total, n ? total / n : 0.0, first_update_ms, ms(t_prog, t_start));
        pct("update", upd);
        pct("dynamics", dyn);
        pct("other", other);
        pct("step", step);
        std::fflush(stdout);
    }
    ~Stats() { report(); }
};
inline Stats& S() {
    static Stats s;
    return s;
}

// Use as part of the demo loop condition: while (... && senprof::cont())
inline bool cont() {
    auto& s = S();
    auto now = clk::now();
    if (!s.started) {
        s.started = true;
        if (s.warm == 0) {
            s.t_start = now;
            s.t_last = now;
        }
    } else {
        s.steps++;  // one iteration completed
        double st = ms(s.step_t0, now);
        if (s.steps > s.warm) {
            s.upd.push_back(s.cur_upd);
            s.dyn.push_back(s.cur_dyn);
            s.other.push_back(st - s.cur_upd - s.cur_dyn);
            s.step.push_back(st);
            s.t_last = now;
        } else if (s.steps == s.warm) {
            s.t_start = now;
            s.t_last = now;
        }
    }
    s.cur_upd = 0;
    s.cur_dyn = 0;
    s.step_t0 = now;
    if (s.max_steps >= 0 && s.steps >= s.max_steps) {
        s.report();
        return false;
    }
    return true;
}
struct Scope {
    double& acc;
    clk::time_point t0;
    Scope(double& a, const char* name) : acc(a), t0(clk::now()) { nvtxRangePushA(name); }
    ~Scope() {
        nvtxRangePop();
        acc += ms(t0, clk::now());
    }
};
}  // namespace senprof

#define SP_UPDATE(expr)                                                           \
    do {                                                                          \
        auto _t0 = senprof::clk::now();                                           \
        {                                                                         \
            senprof::Scope _s(senprof::S().cur_upd, "sensor_update");             \
            expr;                                                                 \
        }                                                                         \
        if (senprof::S().first_update_ms < 0)                                     \
            senprof::S().first_update_ms = senprof::ms(_t0, senprof::clk::now()); \
    } while (0)
#define SP_DYN(expr)                                          \
    do {                                                      \
        senprof::Scope _s(senprof::S().cur_dyn, "dynamics"); \
        expr;                                                 \
    } while (0)
