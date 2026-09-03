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
// Authors: Kyle Sha
// =============================================================================
//
// SHOWCASE: shallow DEPTH OF FIELD via ChPhysCameraSensor.
//
// This is the PHYSICS-BASED camera (ChPhysCameraSensor), the same class the OptiX backend uses -- not the
// backend-specific scene knobs. Bokeh comes from ChFilterPhysCameraDefocusBlur, whose blur diameter derives from
// the real lens geometry and the per-pixel depth:
//
//     blur_diameter[px] = f^2 * |d - U| / (N * C * d * (U - f))
//
// with f = focal length, U = focus distance, N = aperture number (f/D), C = pixel pitch, d = pixel depth.
// A stationary Audi on flat terrain under the shipped sky_2_4k HDR; the camera orbits with the focal plane on
// the car, so the terrain and horizon melt into bokeh while the Audi stays sharp.
// HEADLESS: writes 150 PNGs to SENSOR_OUTPUT/SHOWCASE_PHYSCAM_DOF/ then returns.
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <filesystem>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include "chrono_thirdparty/stb/stb_image_write.h"  // implementation lives in libChrono_sensor

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

// The phys-camera chain ends in an RGBA16 buffer (ChFilterImageHalf4ToRGBA16), which ChFilterSave dumps as a
// raw .bin. The showcase suite wants PNGs, so tone the 16-bit values down to 8 bits here. The sensor
// buffer is bottom-up, so write it through stbi_flip_vertically_on_write exactly as ChFilterSave does.
static void save_rgba16_png(const std::string& path, const UserRGBA16BufferPtr& buf) {
    if (!buf || !buf->Buffer)
        return;
    const size_t count = static_cast<size_t>(buf->Width) * buf->Height;
    std::vector<uint8_t> px(count * 4);
    for (size_t i = 0; i < count; ++i) {
        px[i * 4 + 0] = static_cast<uint8_t>(buf->Buffer[i].R >> 8);
        px[i * 4 + 1] = static_cast<uint8_t>(buf->Buffer[i].G >> 8);
        px[i * 4 + 2] = static_cast<uint8_t>(buf->Buffer[i].B >> 8);
        px[i * 4 + 3] = 255;
    }
    stbi_flip_vertically_on_write(1);
    stbi_write_png(path.c_str(), (int)buf->Width, (int)buf->Height, 4, px.data(), (int)buf->Width * 4);
}

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_PHYSCAM_DOF/";

int main(int argc, char** argv) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.55), QUNIT));
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetWheelVisualizationType(VisualizationType::MESH);
    {
        auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
        audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : audi.GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_TMeasyTire.json"));
                tire->SetStepsize(1e-3);
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    const double radius = 6.0, height = 1.8;
    const ChVector3d look(0, 0, 0.8);
    auto orbitPose = [&](double ang) {
        ChVector3d off(radius * std::cos(ang), radius * std::sin(ang), height);
        ChVector3d d = (look - off).GetNormalized();
        return ChFrame<double>(off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    };

    const unsigned int W = 1280, H = 720;
    auto cam = chrono_types::make_shared<ChPhysCameraSensor>(audi.GetChassisBody(),         // body the camera is attached to
                                                             500.0f,                        // update rate, [Hz]
                                                             orbitPose(0.0),                // offset pose
                                                             W, H,                          // image size, [px]
                                                             CameraLensModelType::PINHOLE,  // lens model
                                                             2,                             // supersampling factor
                                                             false,                         // use_diffuse_reflect (GI)
                                                             true,                          // use_denoiser
                                                             true,                          // use_defocus_blur   <-- the effect this demo shows
                                                             false,                         // use_vignetting
                                                             false,                         // use_aggregator
                                                             false,                         // use_noise
                                                             false,                         // use_expsr_to_dv
                                                             Integrator::LEGACY,
                                                             2.2f,   // gamma (sRGB)
                                                             false,  // use_fog
                                                             false   // use_motion_blur
    );

    // 12 mm lens focused on the car at the 6 m orbit radius, at f/2.0. The blur diameter is <= 1 px (i.e. the
    // filter treats the pixel as sharp) while  |d - U| <= N*C*d*(U - f)/f^2, which for these numbers is
    // d in [4.66 m, 8.41 m]. The car spans roughly 5.1-6.9 m of depth seen side-on from 6 m, so the WHOLE
    // Audi stays sharp, while the terrain in front (d ~ 3 m) and behind (d -> 30 m+) reaches the blur
    // asymptote f^2/(N*C*(U-f)) ~ 3.5 px, scaled by defocus_gain into a 35-41 px bokeh.
    const float focal_length = 0.012f;                                     // [m]
    const float hFOV = (float)(CH_PI / 3);                                 // 60 deg, matching the other demos
    const float sensor_width = 2.f * focal_length * std::tan(hFOV / 2.f);  // [m]
    const float pixel_size = 3.45e-6f;                                     // [m] (Blackfly S BFS-U3-31S4C)

    PhysCameraGainParams gain_params;
    PhysCameraNoiseParams noise_params;
    gain_params.defocus_gain = 10.0f;  // scales the physical blur diameter (bokeh size)
    gain_params.defocus_bias = 0.f;    // [px]
    gain_params.vignetting_gain = 0.f;
    gain_params.aggregator_gain = 1e8f;
    gain_params.expsr2dv_gamma = 1.0f;
    gain_params.expsr2dv_crf_type = 2;  // linear
    gain_params.expsr2dv_gains = {1.0f, 1.0f, 1.0f};
    gain_params.expsr2dv_biases = {0.f, 0.f, 0.f};
    noise_params.FPN_rng_seed = 1234;
    noise_params.dark_currents = {0.f, 0.f, 0.f};
    noise_params.noise_gains = {0.f, 0.f, 0.f};
    noise_params.STD_reads = {0.f, 0.f, 0.f};

    // (aperture_num, expsr_time, ISO, focal_length, focus_dist)
    cam->SetCtrlParameters(2.0f, 0.256f, 100.0f, focal_length, (float)radius);
    cam->SetModelParameters(sensor_width, pixel_size, 1000.f, ChVector3f(1.f, 1.f, 1.f), gain_params, noise_params);
    cam->SetName("physcam_dof");
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
    manager->AddSensor(cam);

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        printf("could not create %s: %s\n", out_dir.c_str(), ec.message().c_str());
        return 1;
    }

    printf("Depth-of-field showcase, ChPhysCameraSensor. PNGs -> %s\n", out_dir.c_str());
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
    int saved = 0;
    DriverInputs in;
    in.m_throttle = 0;
    in.m_steering = 0;
    in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        cam->SetOffsetPose(orbitPose(CH_2PI * f / nframes));  // full orbit (synced with the other demos)
        manager->Update();
        if (auto buf = cam->GetMostRecentBuffer<UserRGBA16BufferPtr>())
            if (buf->Buffer)
                save_rgba16_png(out_dir + "frame_" + std::to_string(saved++) + ".png", buf);
        time += step;
    }
    printf("wrote %d frames\n", saved);
    return 0;
}
