// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Cross-backend parity capture harness (not for upstream).
//
// Renders a fixed set of scenes and dumps raw sensor buffers. The same source is
// compiled against each render backend; the dumps are compared offline.
// =============================================================================

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <functional>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChDataPath.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::sensor;

static std::string g_out = ".";
static std::ofstream g_index;

static const unsigned int W = 480, H = 360;

static void Dump(const std::string& name, const std::string& kind, unsigned int w, unsigned int h, const void* p, size_t bytes) {
    std::string path = g_out + "/" + name + ".bin";
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(p), bytes);
    f.close();
    g_index << name << "," << kind << "," << w << "," << h << "," << bytes << "\n";
    g_index.flush();
    printf("  wrote %-22s %s %ux%u (%zu bytes)\n", name.c_str(), kind.c_str(), w, h, bytes);
    fflush(stdout);
}

template <typename T>
static T Grab(std::shared_ptr<ChSensor> s, std::shared_ptr<ChSensorManager> m, ChSystem& sys) {
    T buf;
    for (int i = 0; i < 600; i++) {
        m->Update();
        sys.DoStepDynamics(2e-3);
        buf = s->GetMostRecentBuffer<T>();
        if (buf && buf->Buffer)
            return buf;
    }
    return buf;
}

static std::shared_ptr<ChVisualMaterial> Mat(ChColor c, float rough, float metal, float opacity = 1.f) {
    auto m = chrono_types::make_shared<ChVisualMaterial>();
    m->SetDiffuseColor(c);
    m->SetSpecularColor({1.f, 1.f, 1.f});
    m->SetRoughness(rough);
    m->SetMetallic(metal);
    if (opacity < 1.f) {
        m->SetOpacity(opacity);
        m->SetIllumination(9);
    }
    return m;
}

static void Paint(std::shared_ptr<ChBody> b, std::shared_ptr<ChVisualMaterial> m) {
    b->GetVisualModel()->GetShapeInstances()[0].shape->AddMaterial(m);
}

static std::shared_ptr<ChBody> Box(ChSystem& sys, ChVector3d size, ChVector3d pos, std::shared_ptr<ChVisualMaterial> m) {
    auto b = chrono_types::make_shared<ChBodyEasyBox>(size.x(), size.y(), size.z(), 1000, true, false);
    b->SetPos(pos);
    b->SetFixed(true);
    sys.Add(b);
    Paint(b, m);
    return b;
}

static std::shared_ptr<ChBody> Sphere(ChSystem& sys, double r, ChVector3d pos, std::shared_ptr<ChVisualMaterial> m) {
    auto b = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
    b->SetPos(pos);
    b->SetFixed(true);
    sys.Add(b);
    Paint(b, m);
    return b;
}

static std::shared_ptr<ChSensorManager> MakeManager(ChSystem& sys, int recursions = 4) {
    auto m = chrono_types::make_shared<ChSensorManager>(&sys);
    m->SetRayRecursions(recursions);
    m->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = {0.05f, 0.06f, 0.08f};
    b.color_horizon = {0.05f, 0.06f, 0.08f};
    m->scene->SetBackground(b);
    return m;
}

// A room shared by several scenarios, so geometry is identical where it should be.
static void Room(ChSystem& sys) {
    Box(sys, {8, 8, .2}, {0, 0, -.1}, Mat({0.72f, 0.72f, 0.72f}, 0.85f, 0.f));   // floor
    Box(sys, {.2, 8, 6}, {4, 0, 3}, Mat({0.75f, 0.75f, 0.78f}, 0.9f, 0.f));      // back wall
    Box(sys, {8, .2, 6}, {0, 4, 3}, Mat({0.75f, 0.2f, 0.2f}, 0.9f, 0.f));        // left wall (red)
    Box(sys, {8, .2, 6}, {0, -4, 3}, Mat({0.2f, 0.65f, 0.25f}, 0.9f, 0.f));      // right wall (green)
}

static ChFrame<double> CamPose() {
    return ChFrame<double>({-6, 0, 2.2}, QuatFromAngleAxis(0, {0, 1, 0}));
}

// A zero-size fixed body used purely as the sensor mount. NB: do not call this Ref() -- with
// using namespace chrono in scope, Eigen::Ref is visible and silently wins the lookup.
static std::shared_ptr<ChBody> SensorMount(ChSystem& sys) {
    auto mount = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
    mount->SetFixed(true);
    sys.Add(mount);
    return mount;
}

// ---------------------------------------------------------------- scenarios

static void ScCamera(const std::string& name, std::function<void(ChSystem&, std::shared_ptr<ChSensorManager>)> setup, bool gi = false, int ss = 2, bool fog = false) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0, 0, 0});
    Room(sys);
    auto mgr = MakeManager(sys, gi ? 6 : 4);
    setup(sys, mgr);
    auto ref = SensorMount(sys);
    auto cam = chrono_types::make_shared<ChCameraSensor>(ref, 30.f, CamPose(), W, H, 1.396f, ss,
                                                         CameraLensModelType::PINHOLE, gi, false, Integrator::LEGACY, 2.2f, fog);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    mgr->AddSensor(cam);
    auto buf = Grab<UserRGBA8BufferPtr>(cam, mgr, sys);
    if (buf && buf->Buffer)
        Dump(name, "rgba8", W, H, buf->Buffer.get(), (size_t)W * H * 4);
    else
        printf("  !! %s produced no buffer\n", name.c_str());
}

// The same room and sphere through a distorted lens, so the lens model itself is what varies.
static void ScCameraLens(const std::string& name, CameraLensModelType lens) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0, 0, 0});
    Room(sys);
    Sphere(sys, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
    Box(sys, {0.5, 0.5, 2.2}, {1.6, -2.2, 1.1}, Mat({0.4f, 0.55f, 0.8f}, 0.5f, 0.f));
    auto mgr = MakeManager(sys);
    mgr->scene->AddPointLight({-2, 2, 5}, {1.f, 1.f, 1.f}, 60.f);
    auto ref = SensorMount(sys);
    auto cam = chrono_types::make_shared<ChCameraSensor>(ref, 30.f, CamPose(), W, H, 1.396f, 2, lens);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    mgr->AddSensor(cam);
    auto buf = Grab<UserRGBA8BufferPtr>(cam, mgr, sys);
    if (buf && buf->Buffer)
        Dump(name, "rgba8", W, H, buf->Buffer.get(), (size_t)W * H * 4);
}

// One lidar geometry, several beam configurations. Divergence and sample_radius exercise the
// multi-sample beam path; the return mode decides how those samples collapse to one range.
static void ScLidar(const std::string& name, LidarBeamShape shape, unsigned int sample_radius, LidarReturnMode mode, float div) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0, 0, 0});
    Room(sys);
    Sphere(sys, 1.0, {0, 0, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
    Box(sys, {1.0, 1.0, 2.0}, {1.2, -2.0, 1.0}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
    Box(sys, {0.3, 2.0, 0.3}, {0.4, 1.8, 2.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));  // thin edge, for multi-return
    auto mgr = MakeManager(sys);
    mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
    auto ref = SensorMount(sys);
    auto li = chrono_types::make_shared<ChLidarSensor>(ref, 10.f, CamPose(), W, H, 1.396f, 0.35f, -0.35f, 40.f, shape,
                                                       sample_radius, div, div, mode);
    li->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    mgr->AddSensor(li);
    auto b = Grab<UserDIBufferPtr>(li, mgr, sys);
    if (b && b->Buffer)
        Dump(name, "f32x2", W, H, b->Buffer.get(), (size_t)W * H * sizeof(PixelDI));
}

int main(int argc, char* argv[]) {
    g_out = (argc > 1) ? argv[1] : ".";
    std::string tag = (argc > 2) ? argv[2] : "unknown";
    g_index.open(g_out + "/index.csv");
    printf("parity capture -> %s (backend tag: %s)\n", g_out.c_str(), tag.c_str());

    // 1. one point light, purely specular/direct
    ScCamera("01_point_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        m->scene->AddPointLight({-2, 2, 5}, {1.f, 1.f, 1.f}, 60.f);
    });

    // 2. spot light
    ScCamera("02_spot_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        m->scene->AddSpotLight({-2, 2, 5}, {1.f, 1.f, 1.f}, 80.f, {0.35f, -0.35f, -1.f}, 0.30f, 0.55f);
    });

    // 3. directional light
    ScCamera("03_directional_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        m->scene->AddDirectionalLight({0.9f, 0.9f, 0.85f}, 0.9f, 2.4f);
    });

    // 4. rectangle (area) light
    ScCamera("04_rect_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        m->scene->AddRectangleLight({-1, 1, 5}, {1.f, 1.f, 1.f}, 70.f, {1.5f, 0, 0}, {0, 1.5f, 0});
    });

    // 5. disk light
    ScCamera("05_disk_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        m->scene->AddDiskLight({-1, 1, 5}, {1.f, 1.f, 1.f}, 70.f, {0.2f, -0.2f, -1.f}, 1.2f);
    });

    // 6. BRDF sweep: roughness across x, metallic across y
    ScCamera("06_brdf_sweep", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 2; j++)
                Sphere(s, 0.55, {0.0, -3.0 + i * 1.5, 0.8 + j * 1.5},
                       Mat({0.9f, 0.75f, 0.35f}, 0.05f + 0.235f * i, j ? 1.f : 0.f));
        m->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        m->scene->AddPointLight({-2, -3, 3}, {0.4f, 0.5f, 0.9f}, 50.f);
    });

    // 7. colored transparent media (illum 9)
    ScCamera("07_glass", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 0.9, {0.0, 1.4, 1.1}, Mat({0.35f, 0.75f, 0.95f}, 0.05f, 0.f, 0.35f));
        Sphere(s, 0.9, {0.0, -1.4, 1.1}, Mat({0.95f, 0.55f, 0.25f}, 0.05f, 0.f, 0.55f));
        Box(s, {0.4, 3.6, 1.6}, {2.2, 0, 0.8}, Mat({0.85f, 0.85f, 0.85f}, 0.9f, 0.f));
        m->scene->AddPointLight({-3, 0, 5}, {1.f, 1.f, 1.f}, 80.f);
    });

    // 8. mixed geometry, several lights at once
    ScCamera("08_multi_light", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 0.8, {0, 1.6, 1.0}, Mat({0.8f, 0.3f, 0.3f}, 0.25f, 0.f));
        Sphere(s, 0.8, {0, -1.6, 1.0}, Mat({0.3f, 0.4f, 0.85f}, 0.65f, 1.f));
        Box(s, {1.2, 1.2, 2.0}, {1.0, 0, 1.0}, Mat({0.8f, 0.8f, 0.4f}, 0.5f, 0.f));
        m->scene->AddPointLight({-3, 3, 5}, {1.f, 0.9f, 0.8f}, 70.f);
        m->scene->AddSpotLight({-2, -3, 5}, {0.6f, 0.8f, 1.f}, 70.f, {0.3f, 0.5f, -1.f}, 0.35f, 0.6f);
        m->scene->AddDirectionalLight({0.35f, 0.35f, 0.4f}, 0.7f, 1.2f);
    });

    // 9. global illumination (stochastic: compared statistically, not pixel-wise)
    ScCamera("09_diffuse_gi", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.85f, 0.85f}, 0.9f, 0.f));
        m->scene->AddRectangleLight({-1, 0, 5.4}, {1.f, 1.f, 1.f}, 90.f, {2.0f, 0, 0}, {0, 2.0f, 0});
    }, /*gi=*/true, /*ss=*/4);

    // 15. albedo texture mapping
    ScCamera("15_textured", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        auto tex = chrono_types::make_shared<ChVisualMaterial>();
        tex->SetDiffuseColor({1.f, 1.f, 1.f});
        tex->SetSpecularColor({1.f, 1.f, 1.f});
        tex->SetRoughness(0.6f);
        tex->SetKdTexture(GetChronoDataFile("textures/redwhite.png"));
        auto b1 = chrono_types::make_shared<ChBodyEasyBox>(1.6, 1.6, 1.6, 1000, true, false);
        b1->SetPos({0, 1.3, 1.0});
        b1->SetRot(QuatFromAngleZ(0.6));
        b1->SetFixed(true);
        s.Add(b1);
        Paint(b1, tex);
        Sphere(s, 0.85, {0, -1.4, 1.0}, Mat({0.9f, 0.9f, 0.9f}, 0.15f, 1.f));
        m->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
    });

    // 16. HDR environment map as background and as the only light
    ScCamera("16_envmap", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.1, {0, 0, 1.3}, Mat({0.95f, 0.95f, 0.95f}, 0.08f, 1.f));
        Background b;
        b.mode = BackgroundMode::ENVIRONMENT_MAP;
        b.env_tex = GetChronoDataFile("sensor/textures/kloppenheim_06_4k.hdr");
        m->scene->SetBackground(b);
        m->scene->AddPointLight({-3, 1, 5}, {0.6f, 0.6f, 0.6f}, 60.f);
    });

    // 17. metallic/roughness workflow (SetUseSpecularWorkflow(false)) -- exercises the branch that
    //     Chrono's default (use_specular_workflow = true) never reaches.
    ScCamera("17_metallic_workflow", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 2; j++) {
                auto mm = chrono_types::make_shared<ChVisualMaterial>();
                mm->SetUseSpecularWorkflow(false);
                mm->SetDiffuseColor({0.9f, 0.75f, 0.35f});
                mm->SetRoughness(0.05f + 0.235f * i);
                mm->SetMetallic(j ? 1.f : 0.f);
                auto b = chrono_types::make_shared<ChBodyEasySphere>(0.55, 1000, true, false);
                b->SetPos({0.0, -3.0 + i * 1.5, 0.8 + j * 1.5});
                b->SetFixed(true);
                s.Add(b);
                Paint(b, mm);
            }
        m->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        m->scene->AddPointLight({-2, -3, 3}, {0.4f, 0.5f, 0.9f}, 50.f);
    });

    // 10-12. depth / normal / segmentation on one shared geometry set
    {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        auto mgr = MakeManager(sys);
        auto s1 = Sphere(sys, 0.9, {0, 1.5, 1.1}, Mat({0.8f, 0.3f, 0.3f}, 0.4f, 0.f));
        auto b1 = Box(sys, {1.0, 1.0, 2.2}, {0.5, -1.5, 1.1}, Mat({0.3f, 0.5f, 0.8f}, 0.4f, 0.f));
        s1->GetVisualModel()->GetShapeInstances()[0].shape->GetMaterial(0)->SetClassID(11);
        s1->GetVisualModel()->GetShapeInstances()[0].shape->GetMaterial(0)->SetInstanceID(101);
        b1->GetVisualModel()->GetShapeInstances()[0].shape->GetMaterial(0)->SetClassID(22);
        b1->GetVisualModel()->GetShapeInstances()[0].shape->GetMaterial(0)->SetInstanceID(202);
        mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        auto ref = SensorMount(sys);

        auto dep = chrono_types::make_shared<ChDepthCamera>(ref, 30.f, CamPose(), W, H, 1.396f, 30.f);
        mgr->AddSensor(dep);
        auto nrm = chrono_types::make_shared<ChNormalCamera>(ref, 30.f, CamPose(), W, H, 1.396f);
        mgr->AddSensor(nrm);
        auto seg = chrono_types::make_shared<ChSegmentationCamera>(ref, 30.f, CamPose(), W, H, 1.396f);
        seg->PushFilter(chrono_types::make_shared<ChFilterSemanticAccess>());
        mgr->AddSensor(seg);

        auto d = Grab<UserDepthBufferPtr>(dep, mgr, sys);
        if (d && d->Buffer) Dump("10_depth", "f32", W, H, d->Buffer.get(), (size_t)W * H * sizeof(PixelDepth));
        auto n = Grab<UserNormalBufferPtr>(nrm, mgr, sys);
        if (n && n->Buffer) Dump("11_normal", "f32x3", W, H, n->Buffer.get(), (size_t)W * H * sizeof(PixelNormal));
        auto g = Grab<UserSemanticBufferPtr>(seg, mgr, sys);
        if (g && g->Buffer) Dump("12_segmentation", "u16x2", W, H, g->Buffer.get(), (size_t)W * H * sizeof(PixelSemantic));
    }

    // 13. lidar range image
    {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        Sphere(sys, 1.0, {0, 0, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
        Box(sys, {1.0, 1.0, 2.0}, {1.2, -2.0, 1.0}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
        auto mgr = MakeManager(sys);
        mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        auto ref = SensorMount(sys);
        auto li = chrono_types::make_shared<ChLidarSensor>(ref, 10.f, CamPose(), W, H, 1.396f, 0.35f, -0.35f, 40.f);
        li->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
        mgr->AddSensor(li);
        auto b = Grab<UserDIBufferPtr>(li, mgr, sys);
        if (b && b->Buffer) Dump("13_lidar", "f32x2", W, H, b->Buffer.get(), (size_t)W * H * sizeof(PixelDI));
    }

    // 14. radar
    {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        Sphere(sys, 1.0, {0, 0, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
        auto mgr = MakeManager(sys);
        mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        auto ref = SensorMount(sys);
        const unsigned int RW = 240, RH = 120;
        auto ra = chrono_types::make_shared<ChRadarSensor>(ref, 10.f, CamPose(), RW, RH, 1.396f, 0.7f, 40.f);
        ra->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());
        mgr->AddSensor(ra);
        auto b = Grab<UserRadarBufferPtr>(ra, mgr, sys);
        if (b && b->Buffer) Dump("14_radar", "radar", RW, RH, b->Buffer.get(), (size_t)RW * RH * sizeof(RadarReturn));
    }


    // 18. physics-based camera. All the optional post stages are off so this isolates the render
    //     path and the RGBA16 readback rather than the defocus/vignette/noise filter chain.
    {
        ChSystemNSC sys; sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        Sphere(sys, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        auto mgr = MakeManager(sys);
        mgr->scene->AddPointLight({-2, 2, 5}, {1.f, 1.f, 1.f}, 60.f);
        auto ref = SensorMount(sys);
        auto pc = chrono_types::make_shared<ChPhysCameraSensor>(ref, 30.f, CamPose(), W, H,
                                                                CameraLensModelType::PINHOLE, 2);
        pc->SetCtrlParameters(4.f, 0.256f, 100.f, 0.012f, 10.f);
        pc->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
        mgr->AddSensor(pc);
        auto b = Grab<UserRGBA16BufferPtr>(pc, mgr, sys);
        if (b && b->Buffer) Dump("18_phys_camera", "rgba16", W, H, b->Buffer.get(), (size_t)W * H * sizeof(PixelRGBA16));
    }

    // ---- 18-20. lens models: the pinhole path is covered above, these are the distorted ones.
    ScCameraLens("19_lens_fov", CameraLensModelType::FOV_LENS);
    ScCameraLens("20_lens_radial", CameraLensModelType::RADIAL);

    // 21. depth camera through a non-pinhole lens
    {
        ChSystemNSC sys; sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        Sphere(sys, 0.9, {0, 1.5, 1.1}, Mat({0.8f, 0.3f, 0.3f}, 0.4f, 0.f));
        auto mgr = MakeManager(sys);
        mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        auto ref = SensorMount(sys);
        auto dep = chrono_types::make_shared<ChDepthCamera>(ref, 30.f, CamPose(), W, H, 1.396f, 30.f,
                                                            CameraLensModelType::FOV_LENS);
        mgr->AddSensor(dep);
        auto d = Grab<UserDepthBufferPtr>(dep, mgr, sys);
        if (d && d->Buffer) Dump("21_depth_fov", "f32", W, H, d->Buffer.get(), (size_t)W * H * sizeof(PixelDepth));
    }

    // 22. lidar with an elliptical beam, multi-sample radius and real divergence
    ScLidar("22_lidar_elliptical", LidarBeamShape::ELLIPTICAL, 2, LidarReturnMode::MEAN_RETURN, 0.003f);
    // 23. same geometry, strongest-return mode and a wider beam
    ScLidar("23_lidar_strongest", LidarBeamShape::RECTANGULAR, 2, LidarReturnMode::STRONGEST_RETURN, 0.01f);

    // 24. radar against MOVING bodies, so the doppler channels carry signal
    {
        ChSystemNSC sys; sys.SetGravitationalAcceleration({0, 0, 0});
        Room(sys);
        auto a = Sphere(sys, 0.8, {0, 1.4, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
        auto b = Sphere(sys, 0.8, {0, -1.4, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.5f, 0.f));
        a->SetFixed(false); b->SetFixed(false);
        a->SetPosDt({3.0, 0, 0});     // approaching the sensor
        b->SetPosDt({-2.0, 1.0, 0});  // receding and crossing
        auto mgr = MakeManager(sys);
        mgr->scene->AddPointLight({-3, 1, 5}, {1.f, 1.f, 1.f}, 80.f);
        auto ref = SensorMount(sys);
        const unsigned int RW = 240, RH = 120;
        auto ra = chrono_types::make_shared<ChRadarSensor>(ref, 10.f, CamPose(), RW, RH, 1.396f, 0.7f, 40.f);
        ra->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());
        mgr->AddSensor(ra);
        auto bf = Grab<UserRadarBufferPtr>(ra, mgr, sys);
        if (bf && bf->Buffer) Dump("24_radar_doppler", "radar", RW, RH, bf->Buffer.get(), (size_t)RW * RH * sizeof(RadarReturn));
    }

    // 25. exponential fog
    ScCamera("25_fog", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        Sphere(s, 1.0, {0, 0, 1.2}, Mat({0.85f, 0.8f, 0.75f}, 0.35f, 0.f));
        Box(s, {0.6, 0.6, 2.4}, {2.5, -2.0, 1.2}, Mat({0.7f, 0.7f, 0.75f}, 0.6f, 0.f));
        m->scene->AddPointLight({-2, 2, 5}, {1.f, 1.f, 1.f}, 60.f);
        m->scene->SetFogColor({0.55f, 0.6f, 0.7f});
        m->scene->SetFogScattering(0.06f);
    }, false, 2, true);

    // 26. emissive materials, which bypass the lighting integral entirely
    ScCamera("26_emissive", [](ChSystem& s, std::shared_ptr<ChSensorManager> m) {
        auto e1 = chrono_types::make_shared<ChVisualMaterial>();
        e1->SetDiffuseColor({0.1f, 0.1f, 0.1f});
        e1->SetEmissiveColor({0.2f, 0.9f, 0.4f});
        e1->SetEmissivePower(4.f);
        auto sp = chrono_types::make_shared<ChBodyEasySphere>(0.9, 1000, true, false);
        sp->SetPos({0, 1.4, 1.2}); sp->SetFixed(true); s.Add(sp); Paint(sp, e1);
        Sphere(s, 0.9, {0, -1.4, 1.2}, Mat({0.8f, 0.8f, 0.8f}, 0.4f, 0.f));
        m->scene->AddPointLight({-3, 0, 5}, {0.5f, 0.5f, 0.5f}, 50.f);
    });

    // 27-29. non-rendered sensors. These do not touch the ray-tracing backend at all, so any
    //        difference would be in the manager, the key-frame plumbing or the RNG streams.
    {
        ChSystemNSC sys; sys.SetGravitationalAcceleration({0, 0, -9.81});
        auto body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
        body->SetPos({0, 0, 2});
        body->SetPosDt({2.0, 0.5, 0});
        body->SetAngVelLocal({0.3, 0.7, 1.1});
        sys.Add(body);
        auto mgr = MakeManager(sys);
        auto none = chrono_types::make_shared<ChNoiseNone>();
        auto acc = chrono_types::make_shared<ChAccelerometerSensor>(body, 100.f, ChFrame<double>(), none);
        acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
        mgr->AddSensor(acc);
        auto gyr = chrono_types::make_shared<ChGyroscopeSensor>(body, 100.f, ChFrame<double>(), none);
        gyr->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
        mgr->AddSensor(gyr);
        auto mag = chrono_types::make_shared<ChMagnetometerSensor>(body, 100.f, ChFrame<double>(), none, ChVector3d(43.07, -89.40, 260.0));
        mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
        mgr->AddSensor(mag);
        auto gps = chrono_types::make_shared<ChGPSSensor>(body, 100.f, ChFrame<double>(), ChVector3d(43.07, -89.40, 260.0), none);
        gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
        mgr->AddSensor(gps);
        auto tach = chrono_types::make_shared<ChTachometerSensor>(body, 100.f, ChFrame<double>(), ChTachometerSensor::Axis::Z);
        tach->PushFilter(chrono_types::make_shared<ChFilterTachometerAccess>());
        mgr->AddSensor(tach);

        auto a = Grab<UserAccelBufferPtr>(acc, mgr, sys);
        if (a && a->Buffer) Dump("27_imu_accel", "f64x3", 1, 1, a->Buffer.get(), sizeof(AccelData));
        auto g = Grab<UserGyroBufferPtr>(gyr, mgr, sys);
        if (g && g->Buffer) Dump("27_imu_gyro", "f64x3", 1, 1, g->Buffer.get(), sizeof(GyroData));
        auto mg = Grab<UserMagnetBufferPtr>(mag, mgr, sys);
        if (mg && mg->Buffer) Dump("27_imu_magnet", "f64x3", 1, 1, mg->Buffer.get(), sizeof(MagnetData));
        auto gp = Grab<UserGPSBufferPtr>(gps, mgr, sys);
        if (gp && gp->Buffer) Dump("28_gps", "f64x3", 1, 1, gp->Buffer.get(), sizeof(GPSData));
        auto tc = Grab<UserTachometerBufferPtr>(tach, mgr, sys);
        if (tc && tc->Buffer) Dump("29_tachometer", "f32", 1, 1, tc->Buffer.get(), sizeof(TachometerData));
    }

    g_index.close();
    printf("done.\n");
    return 0;
}
