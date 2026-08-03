// =============================================================================
// TIER 0 -- GOLDEN-IMAGE RENDERER
// =============================================================================
//
// Renders a small, fast, fully DETERMINISTIC set of frames that the golden-image
// harness (tools/golden.py) pixel-diffs against blessed references.
//
// "Deterministic" is a hard requirement here, and it is enforced by construction:
//   * nothing in the scene moves (zero gravity, every body fixed), and the frame is
//     captured at a fixed step count,
//   * no global illumination, no area lights, no depth of field, no sensor noise and
//     no denoiser -- i.e. none of the stochastic features, all of which live in tier 2,
//   * the Metal shader seeds its RNG from the pixel index rather than from time, so
//     even the supersampled path reproduces exactly frame to frame. That property is
//     asserted directly by
//     utest_SEN_metal_stochastic.MetalStochastic.deterministic_when_all_stochastic_features_are_off.
//
// Everything is drawn from primitives and data that ship with Chrono, so there is no
// external data dependency and the whole run takes well under a second.
//
// Usage:  verify_golden <out_dir> [base|env]
//   base  the 8 primary modalities against a procedural gradient sky   (default)
//   env   2 frames against the shipped HDR environment map, which exercises a
//         completely different code path (texture sampler + image-based reflections)
//
// Each camera writes exactly one PNG, <out_dir>/<name>/frame_0.png. The run also
// writes <out_dir>/signature.txt: a handful of scalar statistics for the lidar and
// radar modalities, which have no image form. Those cost a few hundred bytes instead
// of a binary blob and still catch a regression in the beam models.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/core/ChRotation.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::sensor;

static const unsigned GW = 160, GH = 120;
static const float GHFOV = (float)(CH_PI / 3);
static const float RATE = 1.0f;  // one launch inside the short window stepped below

// A matte/glossy material with segmentation ids, applied to every shape on a body.
static void Paint(std::shared_ptr<ChBody> b,
                  ChColor kd,
                  float roughness,
                  float metallic,
                  unsigned short cls,
                  unsigned short inst,
                  float opacity = 1.f,
                  const std::string& tex = "") {
    auto m = chrono_types::make_shared<ChVisualMaterial>();
    m->SetDiffuseColor(kd);
    m->SetSpecularColor({0.2f, 0.2f, 0.2f});
    m->SetRoughness(roughness);
    m->SetMetallic(metallic);
    m->SetOpacity(opacity);
    m->SetClassID(cls);
    m->SetInstanceID(inst);
    if (!tex.empty())
        m->SetKdTexture(tex);
    if (auto vm = b->GetVisualModel())
        for (auto& si : vm->GetShapeInstances()) {
            si.shape->GetMaterials().clear();
            si.shape->AddMaterial(m);
        }
}

int main(int argc, char** argv) {

    std::string out = (argc > 1) ? std::string(argv[1]) : std::string("golden_out");
    if (!out.empty() && out.back() != '/')
        out += '/';
    const std::string mode = (argc > 2) ? std::string(argv[2]) : std::string("base");
    const bool env_mode = (mode == "env");

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));  // nothing may move

    // ---------------------------------------------------------------- scene
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(60, 60, 0.4, 1000, true, false);
    ground->SetPos({0, 0, -0.2});
    ground->SetFixed(true);
    sys.Add(ground);
    Paint(ground, ChColor(0.55f, 0.55f, 0.58f), 0.9f, 0.f, 1, 101, 1.f,
          GetChronoDataFile("sensor/textures/checkerboard.png"));

    auto redbox = chrono_types::make_shared<ChBodyEasyBox>(1.2, 1.2, 1.2, 1000, true, false);
    redbox->SetPos({6.0, -1.4, 0.6});
    redbox->SetFixed(true);
    sys.Add(redbox);
    Paint(redbox, ChColor(0.75f, 0.12f, 0.10f), 0.55f, 0.f, 2, 102);

    auto ball = chrono_types::make_shared<ChBodyEasySphere>(0.8, 1000, true, false);
    ball->SetPos({7.5, 1.6, 0.8});
    ball->SetFixed(true);
    sys.Add(ball);
    Paint(ball, ChColor(0.15f, 0.62f, 0.22f), env_mode ? 0.05f : 0.25f, env_mode ? 1.0f : 0.0f, 3, 103);

    auto post = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, 0.28, 2.6, 1000, true, false);
    post->SetPos({4.2, 0.9, 1.3});
    post->SetFixed(true);
    sys.Add(post);
    Paint(post, ChColor(0.25f, 0.32f, 0.75f), 0.4f, 0.2f, 4, 104);

    // a semi-transparent slab, so the alpha-composited transmission path is covered too
    auto glass = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1.6, 1.4, 1000, true, false);
    glass->SetPos({3.4, -1.9, 0.7});
    glass->SetFixed(true);
    sys.Add(glass);
    Paint(glass, ChColor(0.85f, 0.9f, 0.95f), 0.08f, 0.f, 5, 105, 0.35f);

    auto mount = chrono_types::make_shared<ChBody>();
    mount->SetFixed(true);
    sys.Add(mount);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->SetAmbientLight({0.18f, 0.18f, 0.20f});
    manager->scene->AddDirectionalLight(ChColor(1.0f, 0.97f, 0.92f), 0.9f, 0.75f);
    manager->scene->AddPointLight({2.f, 4.f, 5.f}, ChColor(0.9f, 0.9f, 1.0f), 40.f);

    if (env_mode) {
        manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"), 1.0f);
    } else {
        Background bg;
        bg.mode = BackgroundMode::GRADIENT;
        bg.color_zenith = {0.22f, 0.38f, 0.72f};
        bg.color_horizon = {0.75f, 0.80f, 0.88f};
        manager->scene->SetBackground(bg);
        manager->scene->SetFogColor({0.7f, 0.72f, 0.78f});
        manager->scene->SetFogScatteringFromDistance(12.0);
    }

    // one fixed viewpoint for every camera: 1.9 m up, behind the props, tilted 7 deg down
    const ChFrame<double> pose(ChVector3d(-1.2, -0.3, 1.9), QuatFromAngleY(7.0 * CH_DEG_TO_RAD));

    std::vector<std::shared_ptr<ChSensor>> sensors;
    auto add_cam = [&](const std::string& name, unsigned ss, CameraLensModelType lens, bool fog,
                       ChVector3f distortion = ChVector3f(0, 0, 0)) {
        auto c = chrono_types::make_shared<ChCameraSensor>(mount, RATE, pose, GW, GH, GHFOV, ss, lens, false, false,
                                                           Integrator::LEGACY, 2.2f, fog);
        c->SetName(name);
        c->SetLag(0);
        c->SetCollectionWindow(0);
        if (lens == CameraLensModelType::RADIAL)
            c->SetRadialLensParameters(distortion);
        c->PushFilter(chrono_types::make_shared<ChFilterSave>(out + name + "/"));
        manager->AddSensor(c);
        sensors.push_back(c);
    };

    UserDIBufferPtr lidar_buf;
    UserRadarBufferPtr radar_buf;
    std::shared_ptr<ChLidarSensor> lidar;
    std::shared_ptr<ChRadarSensor> radar;

    if (env_mode) {
        add_cam("env_rgb", 1, CameraLensModelType::PINHOLE, false);
        add_cam("env_rgb_ss2", 2, CameraLensModelType::PINHOLE, false);
    } else {
        add_cam("rgb", 1, CameraLensModelType::PINHOLE, false);
        add_cam("rgb_ss2", 2, CameraLensModelType::PINHOLE, false);
        add_cam("fisheye", 1, CameraLensModelType::FOV_LENS, false);
        add_cam("radial", 1, CameraLensModelType::RADIAL, false, ChVector3f(-0.22f, 0.05f, 0.f));
        add_cam("fog", 1, CameraLensModelType::PINHOLE, true);

        auto dep = chrono_types::make_shared<ChDepthCamera>(mount, RATE, pose, GW, GH, GHFOV, 30.f);
        dep->SetName("depth");
        dep->SetLag(0);
        dep->SetCollectionWindow(0);
        dep->PushFilter(chrono_types::make_shared<ChFilterSave>(out + "depth/"));
        manager->AddSensor(dep);
        sensors.push_back(dep);

        auto nrm = chrono_types::make_shared<ChNormalCamera>(mount, RATE, pose, GW, GH, GHFOV);
        nrm->SetName("normal");
        nrm->SetLag(0);
        nrm->SetCollectionWindow(0);
        nrm->PushFilter(chrono_types::make_shared<ChFilterSave>(out + "normal/"));
        manager->AddSensor(nrm);
        sensors.push_back(nrm);

        auto seg = chrono_types::make_shared<ChSegmentationCamera>(mount, RATE, pose, GW, GH, GHFOV);
        seg->SetName("segmentation");
        seg->SetLag(0);
        seg->SetCollectionWindow(0);
        seg->PushFilter(chrono_types::make_shared<ChFilterSave>(out + "segmentation/"));
        manager->AddSensor(seg);
        sensors.push_back(seg);

        lidar = chrono_types::make_shared<ChLidarSensor>(mount, RATE, pose, 64, 8, 0.9f, 0.15f, -0.35f, 60.f);
        lidar->SetName("lidar");
        lidar->SetLag(0);
        lidar->SetCollectionWindow(0);
        lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
        manager->AddSensor(lidar);
        sensors.push_back(lidar);

        radar = chrono_types::make_shared<ChRadarSensor>(mount, RATE, pose, 64, 8, 0.9f, 0.5f, 60.f);
        radar->SetName("radar");
        radar->SetLag(0);
        radar->SetCollectionWindow(0);
        radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());
        manager->AddSensor(radar);
        sensors.push_back(radar);
    }

    // Step a window shorter than one update period so every sensor launches exactly once.
    for (int i = 0; i < 12; ++i) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }
    if (lidar)
        lidar_buf = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    if (radar)
        radar_buf = radar->GetMostRecentBuffer<UserRadarBufferPtr>();

    int launched_wrong = 0;
    for (auto& s : sensors)
        if (s->GetNumLaunches() != 1) {
            printf("golden: sensor %s launched %u times, expected 1\n", s->GetName().c_str(), s->GetNumLaunches());
            launched_wrong++;
        }

    // ------------------------------------------------- non-image signatures
    if (!env_mode) {
        FILE* f = fopen((out + "signature.txt").c_str(), "w");
        if (!f) {
            printf("golden: cannot write %ssignature.txt\n", out.c_str());
            return 1;
        }
        auto stats = [&](const char* label, const float* v, size_t n) {
            double lo = 1e30, hi = -1e30, sum = 0;
            long hits = 0;
            for (size_t i = 0; i < n; ++i) {
                if (v[i] <= 0.f)
                    continue;
                hits++;
                sum += v[i];
                lo = std::min(lo, (double)v[i]);
                hi = std::max(hi, (double)v[i]);
            }
            fprintf(f, "%s.returns %ld\n", label, hits);
            fprintf(f, "%s.min %.5f\n", label, hits ? lo : 0.0);
            fprintf(f, "%s.max %.5f\n", label, hits ? hi : 0.0);
            fprintf(f, "%s.mean %.5f\n", label, hits ? sum / hits : 0.0);
        };
        if (lidar_buf && lidar_buf->Buffer) {
            const size_t n = (size_t)64 * 8;
            std::vector<float> r(n), it(n);
            for (size_t i = 0; i < n; ++i) {
                r[i] = lidar_buf->Buffer[i].range;
                it[i] = lidar_buf->Buffer[i].intensity;
            }
            stats("lidar.range", r.data(), n);
            stats("lidar.intensity", it.data(), n);
        }
        if (radar_buf && radar_buf->Buffer) {
            const size_t n = (size_t)64 * 8;
            std::vector<float> r(n), a(n);
            for (size_t i = 0; i < n; ++i) {
                r[i] = radar_buf->Buffer[i].range;
                a[i] = radar_buf->Buffer[i].amplitude;
            }
            stats("radar.range", r.data(), n);
            stats("radar.amplitude", a.data(), n);
        }
        fclose(f);
    }

    printf("golden[%s]: wrote %zu sensor outputs to %s\n", mode.c_str(), sensors.size(), out.c_str());
    return launched_wrong == 0 ? 0 : 1;
}
