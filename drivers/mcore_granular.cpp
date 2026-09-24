// Scalable headless granular driver for Chrono::Multicore profiling (campaign tool, not part of Chrono).
// Env: PROF_METHOD=SMC|NSC  PROF_N=<approx bodies>  PROF_SHAPE=sphere|box|mixed  PROF_THREADS  PROF_WARM  PROF_STEPS
//      PROF_BINS=<bins per axis, 0 = FIXED_DENSITY grid (density PROF_DENS, default 5)>  PROF_SOLVER=APGD|APGDREF|BB
#include <cmath>
#include <cstring>
#include <random>
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "prof.h"
#include "chrono_multicore/constraints/ChConstraintRigidRigid.h"
using namespace chrono;

// Replicates ChIterativeSolverMulticoreNSC::ComputeD phase by phase with timers (SLIDING mode, rigid contacts only).
static void ProbeComputeD(ChSystemMulticore* sys) {
    auto* dm = sys->data_manager;
    using clk = std::chrono::steady_clock;
    auto ms = [](clk::time_point a, clk::time_point b) { return 1e3 * std::chrono::duration<double>(b - a).count(); };
    for (int rep = 0; rep < 3; rep++) {
        uint nc = dm->cd_data->num_rigid_contacts;
        int num_rows = 3 * nc;
        auto& D_T = dm->host_data.D_T;
        auto t0 = clk::now();
        D_T.resize(num_rows, dm->num_dof);
        Eigen::VectorXi d_nnz(num_rows);
        d_nnz.setConstant(12);
        D_T.reserve(d_nnz);
        auto t1 = clk::now();
        dm->rigid_rigid->GenerateSparsity();
        auto t2 = clk::now();
        dm->host_data.b.resize(dm->num_constraints);
        dm->host_data.b.setZero();
        dm->rigid_rigid->Build_D();
        auto t3 = clk::now();
        D_T.makeCompressed();
        auto t4 = clk::now();
        dm->host_data.D = D_T.transpose();
        auto t5 = clk::now();
        dm->host_data.M_invD = dm->host_data.D;
        auto t6 = clk::now();
        std::printf("PROBE_D rep=%d contacts=%u rows=%d dof=%u nnz=%ld reserve_ms=%.3f gensparsity_ms=%.3f buildD_ms=%.3f "
                    "compress_ms=%.3f transpose_ms=%.3f copyMinvD_ms=%.3f\n",
                    rep, nc, num_rows, dm->num_dof, (long)D_T.nonZeros(), ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4),
                    ms(t4, t5), ms(t5, t6));
    }
}


int main() {
    const char* m = std::getenv("PROF_METHOD");
    bool smc = !(m && std::strcmp(m, "NSC") == 0);
    const char* shp = std::getenv("PROF_SHAPE");
    std::string shape = shp ? shp : "sphere";
    int N = prof::env_int("PROF_N", 20000);
    int threads = prof::threads(8);
    int bins = prof::env_int("PROF_BINS", 0);

    double r = 0.01;
    // SMC: touching lattice with small jitter so contacts exist from step 0; NSC: 2.5% gap (inside the envelope)
    double spacing = (smc ? 2.0 : 2.05) * r;
    // column footprint nx x nx, height nz ~ 2 nx
    int nx = std::max(2, (int)std::round(std::cbrt(N / 2.0)));
    int nz = std::max(2, N / (nx * nx));
    double hx = nx * spacing / 2 + r;
    double time_step = smc ? 1e-4 : 1e-3;

    ChSystemMulticore* sys;
    std::shared_ptr<ChContactMaterial> mat;
    if (smc) {
        auto s = new ChSystemMulticoreSMC();
        s->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
        s->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;
        auto mm = chrono_types::make_shared<ChContactMaterialSMC>();
        mm->SetYoungModulus(1e7f);
        mm->SetFriction(0.4f);
        mm->SetRestitution(0.1f);
        mat = mm;
        sys = s;
    } else {
        auto s = new ChSystemMulticoreNSC();
        s->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
        s->GetSettings()->solver.max_iteration_normal = 0;
        s->GetSettings()->solver.max_iteration_sliding = prof::env_int("PROF_ITERS", 100);
        s->GetSettings()->solver.max_iteration_spinning = 0;
        s->GetSettings()->solver.max_iteration_bilateral = 0;
        s->GetSettings()->solver.alpha = 0;
        s->GetSettings()->solver.contact_recovery_speed = 0.1;
        s->GetSettings()->solver.tolerance = 1e-5;
        const char* sv = std::getenv("PROF_SOLVER");
        std::string solver = sv ? sv : "APGD";
        s->ChangeSolverType(solver == "BB" ? SolverType::BB
                                           : (solver == "APGDREF" ? SolverType::APGDREF : SolverType::APGD));
        s->GetSettings()->collision.collision_envelope = 0.05 * r;
        auto mm = chrono_types::make_shared<ChContactMaterialNSC>();
        mm->SetFriction(0.4f);
        mat = mm;
        sys = s;
    }
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->SetNumThreads(threads);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    if (bins > 0) {
        sys->GetSettings()->collision.bins_per_axis = vec3(bins, bins, bins);
    } else {
        sys->GetSettings()->collision.broadphase_grid = ChBroadphase::GridType::FIXED_DENSITY;
        sys->GetSettings()->collision.grid_density = prof::env_int("PROF_DENS", 5);
    }

    // container (no top)
    utils::CreateBoxContainer(sys, mat, ChVector3d(2 * hx, 2 * hx, nz * spacing * 1.5), 0.05, VNULL, QUNIT, true,
                              true, false, true);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> jit(smc ? -0.005 * r : -0.02 * r, smc ? 0.005 * r : 0.02 * r);
    double mass = 1000 * CH_4_3 * CH_PI * r * r * r;
    int count = 0;
    for (int k = 0; k < nz; k++)
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nx; i++) {
                ChVector3d pos(-hx + r + i * spacing + jit(rng), -hx + r + j * spacing + jit(rng),
                               r + (smc ? 0.0 : 0.001) + k * spacing + jit(rng));
                auto b = chrono_types::make_shared<ChBody>();
                b->SetMass(mass);
                b->SetInertiaXX(0.4 * mass * r * r * ChVector3d(1, 1, 1));
                b->SetPos(pos);
                b->EnableCollision(true);
                bool box = shape == "box" || (shape == "mixed" && (count % 2));
                if (box)
                    utils::AddBoxGeometry(b.get(), mat, ChVector3d(1.4 * r, 1.4 * r, 1.4 * r));
                else
                    utils::AddSphereGeometry(b.get(), mat, r);
                sys->AddBody(b);
                count++;
            }
    std::printf("mcore_granular method=%s shape=%s bodies=%d nx=%d nz=%d threads=%d bins=%d dt=%g\n",
                smc ? "SMC" : "NSC", shape.c_str(), count, nx, nz, threads, bins, time_step);
    std::fflush(stdout);
    int probe_at = prof::env_int("PROF_PROBE_D", -1);
    int probe_rc = prof::env_int("PROF_PROBE_RC", -1);
    for (int k = 0;; k++) {
        if (k == probe_at && !smc)
            ProbeComputeD(sys);
        if (k == probe_rc) {
            // time ReportContacts (per-contact composite material + container fill) in isolation, 20 reps
            auto cs = sys->GetCollisionSystem();
            auto cc = sys->GetContactContainer();
            std::vector<double> t;
            for (int rep = 0; rep < 20; rep++) {
                auto a = std::chrono::steady_clock::now();
                cs->ReportContacts(cc.get());
                t.push_back(1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count());
            }
            std::sort(t.begin(), t.end());
            std::printf("PROBE_RC threads=%d contacts=%u median_ms=%.4f min_ms=%.4f\n", threads, sys->GetNumContacts(), t[10], t[0]);
        }
        prof::step(sys, time_step);
    }
}
