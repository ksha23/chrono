// Validation driver for fix/sensor-physcam-params (derived from demo_SEN_phys_cam, headless).
// usage: physcam_drv W H steps warmup rate_hz lidar(0/1) cam(0/1) mem_every hash(0/1) [dump_frame dump_prefix]
#include "senprof.h"
#include <cuda_runtime_api.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
using namespace chrono;
using namespace chrono::sensor;

static uint64_t fnv(const void* p, size_t n, uint64_t h = 1469598103934665603ull) {
    const unsigned char* c = (const unsigned char*)p;
    for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ull; }
    return h;
}
static long proc_gpu_mib() {  // this process's device memory as reported by the driver
    char cmd[256];
    snprintf(cmd, sizeof cmd, "nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits | awk -F, '$1==%d{print $2}'", (int)getpid());
    FILE* f = popen(cmd, "r"); long v = -1; if (f) { if (fscanf(f, "%ld", &v) != 1) v = -1; pclose(f); } return v;
}

int main(int argc, char* argv[]) {
    unsigned int image_width = argc > 1 ? atoi(argv[1]) : 1280;
    unsigned int image_height = argc > 2 ? atoi(argv[2]) : 720;
    long steps = argc > 3 ? atol(argv[3]) : 200;
    long warm = argc > 4 ? atol(argv[4]) : 20;
    float update_rate = argc > 5 ? (float)atof(argv[5]) : 5.f;
    bool use_lidar = argc > 6 && atoi(argv[6]);
    bool use_cam = argc > 7 ? atoi(argv[7]) != 0 : true;
    long mem_every = argc > 8 ? atol(argv[8]) : 0;
    bool do_hash = argc > 9 ? atoi(argv[9]) != 0 : true;
    long dump_frame = argc > 10 ? atol(argv[10]) : -1;
    std::string dump_prefix = argc > 11 ? argv[11] : "frame";
    setenv("SENPROF_STEPS", std::to_string(steps).c_str(), 1);
    setenv("SENPROF_WARMUP", std::to_string(warm).c_str(), 1);
    ChSensorManager::SetRandomSeed(42);
    float fov = (float)CH_PI_3; int alias_factor = 4; bool use_diffuse = false;
    Integrator integrator = Integrator::PATH; CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
    float lag = 0.f, exposure_time = 0.f;  // every step whose time is due renders; no lag
    double step_size = 1e-2;

    ChSystemNSC sys;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({-6, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, 0, 0)));
    mesh_body->SetFixed(true);
    sys.Add(mesh_body);

    auto vis_mat3 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat3->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat3->SetDiffuseColor({.5, .5, .5});
    vis_mat3->SetSpecularColor({.0f, .0f, .0f});
    vis_mat3->SetUseSpecularWorkflow(true);
    vis_mat3->SetClassID(30000);
    vis_mat3->SetInstanceID(30000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, .1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetFixed(true);
    sys.Add(floor);
    {
        auto shape = floor->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat3);
        } else {
            shape->GetMaterials()[0] = vis_mat3;
        }
    }

    // add box object
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    box_body->SetPos({0, -5, 0});
    box_body->SetFixed(true);
    sys.Add(box_body);
    {
        auto shape = box_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat);
        } else {
            shape->GetMaterials()[0] = vis_mat;
        }
    }

    // add sphere object
    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat2->SetDiffuseColor({1.0, 0.0, 0.0});
    vis_mat2->SetSpecularColor({.0f, .0f, .0f});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(0.5f);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({0, 0, 0});
    sphere_body->SetFixed(true);
    sys.Add(sphere_body);
    {
        auto shape = sphere_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat2);
        } else {
            shape->GetMaterials()[0] = vis_mat2;
        }
    }

    // add cylinder object
    auto vis_mat4 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat4->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat4->SetDiffuseColor({0.0, 0.0, 1.0});
    vis_mat4->SetSpecularColor({.0f, .0f, .0f});
    vis_mat4->SetUseSpecularWorkflow(true);
    vis_mat4->SetRoughness(0.5f);
    vis_mat4->SetClassID(30000);
    vis_mat4->SetInstanceID(1000);

    auto cyl_body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, .25, 1, 1000, true, false);
    cyl_body->SetPos({0, 5, 0});
    cyl_body->SetFixed(true);
    sys.Add(cyl_body);
    {
        auto shape = cyl_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat4);
        } else {
            shape->GetMaterials()[0] = vis_mat4;
        }
    }

    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(true);
    sys.Add(ground_body);


    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1.f, 1.f, 1.f}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);
    chrono::ChFrame<double> offset_pose({5, 0, 0}, QuatFromAngleAxis(CH_PI, {0, 0, 1}));

    std::shared_ptr<ChCameraSensor> cam;
    if (use_cam) {
        cam = chrono_types::make_shared<ChCameraSensor>(ground_body, update_rate, offset_pose, image_width, image_height,
                                                        fov, alias_factor, lens_model, use_diffuse, use_diffuse,
                                                        integrator, 2.2, false);
        cam->SetName("Camera Sensor"); cam->SetLag(lag); cam->SetCollectionWindow(exposure_time);
        cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        manager->AddSensor(cam);
    }
    std::shared_ptr<ChLidarSensor> lidar;
    if (use_lidar) {
        lidar = chrono_types::make_shared<ChLidarSensor>(ground_body, update_rate, offset_pose, 923, 23,
                                                         2.f * (float)CH_PI_3, (float)CH_PI / 8.0f, -(float)CH_PI / 8.0f, 100.0f);
        lidar->SetName("Lidar Sensor"); lidar->SetLag(lag); lidar->SetCollectionWindow(exposure_time);
        lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
        manager->AddSensor(lidar);
    }
    auto phys_cam = chrono_types::make_shared<ChPhysCameraSensor>(ground_body, update_rate, offset_pose, image_width,
        image_height, lens_model, alias_factor, use_diffuse, use_diffuse, true, true, true, true, true, integrator,
        2.2f, false, false);
    float phys_cam_px_size = 3.45e-6f, max_scene_light_amount = 1000.0;
    float phys_cam_focal_length = 0.012f, phys_cam_hFOV = fov;
    float phys_cam_sensor_width = 2 * phys_cam_focal_length * tanf(phys_cam_hFOV / 2);
    ChVector3f phys_cam_distort_params(-0.16f, 0.2f, -0.12f);
    PhysCameraGainParams g; PhysCameraNoiseParams n;
    g.defocus_gain = 10.0f; g.defocus_bias = 0.f; g.vignetting_gain = 0.6f; g.aggregator_gain = 1e8f;
    g.expsr2dv_gains = {1.0f, 1.0f, 1.0f}; g.expsr2dv_gamma = 0.f; g.expsr2dv_crf_type = 2;
    ChVector3f phys_cam_rgb_QE_vec(0.4453f, 0.5621f, 0.4713f);
    g.expsr2dv_biases = {0.02f, 0.04f, 0.2f};
    n.FPN_rng_seed = 1234;
    n.dark_currents = {0.000166311f, 0.000341295f, 0.000680946f};
    n.noise_gains = {0.67f * 0.00182512f, 0.67f * 0.00215293f, 0.67f * 0.00318984f};
    n.STD_reads = {0.67f * 2.56849e-05f, 0.67f * 4.08999e-05f, 0.67f * 8.33132e-05f};
    phys_cam->SetCtrlParameters(4.0f, 0.256f, 100.0f, 0.012f, 10.0f);
    phys_cam->SetModelParameters(phys_cam_sensor_width, phys_cam_px_size, max_scene_light_amount, phys_cam_rgb_QE_vec, g, n);
    phys_cam->SetRadialLensParameters(phys_cam_distort_params);
    phys_cam->SetName("Phys Camera Sensor"); phys_cam->SetLag(lag); phys_cam->SetCollectionWindow(exposure_time);
    phys_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
    manager->AddSensor(phys_cam);

    SP_UPDATE(manager->Update());
    float orbit_radius = 10.0f, orbit_rate = 0.2f, ch_time = 0.0f;
    unsigned int last_phys = 0, last_cam = 0, last_lidar = 0;
    uint64_t hphys = 1469598103934665603ull, hcam = hphys, hlidar = hphys;
    long nphys = 0, ncam = 0, nlidar = 0;
    size_t free0 = 0, tot = 0; cudaMemGetInfo(&free0, &tot); long proc0 = -1;
    while (senprof::cont()) {
        float a = ch_time * orbit_rate;
        ChFrame<double> pose({orbit_radius * cos(a), orbit_radius * sin(a), 2}, QuatFromAngleAxis(a + CH_PI, {0, 0, 1}));
        if (cam) cam->SetOffsetPose(pose);
        phys_cam->SetOffsetPose(pose);
        if (do_hash || dump_frame >= 0) {
            auto p = phys_cam->GetMostRecentBuffer<UserRGBA16BufferPtr>();
            if (p && p->Buffer && p->LaunchedCount != last_phys) {
                last_phys = p->LaunchedCount; nphys++;
                uint64_t fh = fnv(p->Buffer.get(), sizeof(PixelRGBA16) * p->Width * p->Height);
                hphys = fnv(&fh, 8, hphys);
                if (nphys <= 3 || nphys % 50 == 0) printf("PHYSFRAME %ld launch=%u hash=%016llx\n", nphys, last_phys, (unsigned long long)fh);
                if (nphys == dump_frame) {
                    std::ofstream o(dump_prefix + ".ppm", std::ios::binary);
                    o << "P6\n" << p->Width << " " << p->Height << "\n255\n";
                    for (unsigned int i = 0; i < p->Width * p->Height; ++i) {
                        unsigned char c[3] = {(unsigned char)(p->Buffer[i].R >> 8), (unsigned char)(p->Buffer[i].G >> 8), (unsigned char)(p->Buffer[i].B >> 8)};
                        o.write((char*)c, 3);
                    }
                }
            }
            if (cam) { auto c = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
                if (c && c->Buffer && c->LaunchedCount != last_cam) { last_cam = c->LaunchedCount; ncam++;
                    hcam = fnv(c->Buffer.get(), sizeof(PixelRGBA8) * c->Width * c->Height, hcam); } }
            if (lidar) { auto l = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
                if (l && l->Buffer && l->LaunchedCount != last_lidar) { last_lidar = l->LaunchedCount; nlidar++;
                    hlidar = fnv(l->Buffer.get(), sizeof(PixelDI) * l->Width * l->Height, hlidar); } }
        }
        if (mem_every > 0 && senprof::S().steps % mem_every == 0) {
            size_t fr = 0; cudaMemGetInfo(&fr, &tot); long pm = proc_gpu_mib(); if (proc0 < 0) proc0 = pm;
            printf("MEM step=%ld phys_launches=%u dev_free_delta_MiB=%.2f proc_MiB=%ld\n", senprof::S().steps,
                   phys_cam->GetNumLaunches(), ((double)free0 - (double)fr) / 1048576.0, pm);
            fflush(stdout);
        }
        SP_UPDATE(manager->Update());
        SP_DYN(sys.DoStepDynamics(step_size));
        ch_time = (float)sys.GetChTime();
    }
    printf("HASH phys n=%ld %016llx cam n=%ld %016llx lidar n=%ld %016llx\n", nphys, (unsigned long long)hphys, ncam,
           (unsigned long long)hcam, nlidar, (unsigned long long)hlidar);
    printf("LAUNCHES phys=%u\n", phys_cam->GetNumLaunches());
    return 0;
}
