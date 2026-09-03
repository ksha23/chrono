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
// SHOWCASE: full PHYSICS-BASED SENSOR chain via ChPhysCameraSensor.
// Strong optical cos^4 vignette + real sensor grain, driven entirely by the physical camera parameters.
//
// ChPhysCameraSensor, running all four post-render stages on whichever ray-tracing backend is built:
//
//   ChFilterPhysCameraVignetting  E <- E * (1 - G + G * cos^4(theta))          (cos^4 illumination falloff)
//   ChFilterPhysCameraAggregator  E <- E * G * P / N^2 * C^2 * t * QE          (irradiance -> electrons)
//   ChFilterPhysCameraNoise       I  = L + N_read, L ~ Poisson(mu + D*t)       (shot / dark / read / FPN noise)
//   ChFilterPhysCameraExpsrToDV   I  = a * ISO * E + b                         (camera response function)
//
// Camera model parameters are those of a Blackfly S BFS-U3-31S4C with a 12 mm lens, matching
// src/demos/sensor/demo_SEN_phys_cam.cpp, so the identical setup runs on either backend.
// A stationary Audi on flat terrain under the shipped sky_2_4k HDR, camera orbiting.
// HEADLESS: writes 150 PNGs to SENSOR_OUTPUT/SHOWCASE_PHYSCAM_GRAIN/ then returns.
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

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_PHYSCAM_GRAIN/";

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
                                                             1,                             // supersampling factor
                                                             false,                         // use_diffuse_reflect (GI)
                                                             false,                         // use_denoiser
                                                             false,                         // use_defocus_blur
                                                             true,                          // use_vignetting     <-- cos^4 falloff
                                                             true,                          // use_aggregator     <-- irradiance -> electrons
                                                             true,                          // use_noise          <-- shot / dark / read / FPN
                                                             true,                          // use_expsr_to_dv    <-- camera response function
                                                             Integrator::LEGACY,
                                                             2.2f,   // gamma (sRGB)
                                                             false,  // use_fog
                                                             false   // use_motion_blur
    );

    // Camera model parameters: Blackfly S BFS-U3-31S4C, 12 mm lens (as in demo_SEN_phys_cam.cpp).
    const float focal_length = 0.012f;                                     // [m]
    const float hFOV = (float)(CH_PI / 3);                                 // 60 deg, matching the other demos
    const float sensor_width = 2.f * focal_length * std::tan(hFOV / 2.f);  // [m]
    const float pixel_size = 3.45e-6f;                                     // [m]
    const float max_scene_light_amount = 1000.f;                           // [lux], distance-diminished

    PhysCameraGainParams gain_params;
    PhysCameraNoiseParams noise_params;
    gain_params.defocus_gain = 0.f;
    gain_params.defocus_bias = 0.f;
    // vignetting_gain blends between "no vignette" (0) and the bare optical cos^4 law (1):
    //   gain = 1 - G + G * cos^4(theta).
    // G = 1 is therefore the physically pure setting, and it is what this demo wants -- any G < 1 is a fudge
    // that lifts the corners back toward the centre. The corner field angle follows from the FOV alone:
    // sensor_width = 2*f*tan(hFOV/2), so theta_corner = atan(1.147*tan(hFOV/2)) and the focal length cancels.
    // At the suite's standard 60 deg hFOV that is 33.5 deg, giving cos^4 = 0.483 -- the frame corners land at
    // ~48% of centre. (A wider lens would darken them further: 75 deg -> 0.32, 90 deg -> 0.19.)
    gain_params.vignetting_gain = 1.0f;
    gain_params.aggregator_gain = 1e8f;
    gain_params.expsr2dv_gamma = 1.0f;
    gain_params.expsr2dv_crf_type = 2;  // 0: gamma_correct, 1: sigmoid, 2: linear
    gain_params.expsr2dv_gains = {1.0f, 1.0f, 1.0f};
    gain_params.expsr2dv_biases = {0.003f, 0.006f, 0.014f};  // slight cool cast in the shadows, as on the real sensor
    ChVector3f rgb_QE_vec(0.4453f, 0.5621f, 0.4713f);        // measured RGB quantum efficiencies

    // Measured noise figures, scaled up 3x so the grain is actually visible in a showcase animation.
    noise_params.FPN_rng_seed = 1234;
    noise_params.dark_currents = {0.000166311f, 0.000341295f, 0.000680946f};  // [electrons/sec]
    noise_params.noise_gains = {3.f * 0.00182512f, 3.f * 0.00215293f, 3.f * 0.00318984f};
    noise_params.STD_reads = {3.f * 2.56849e-05f, 3.f * 4.08999e-05f, 3.f * 8.33132e-05f};

    // (aperture_num, expsr_time, ISO, focal_length, focus_dist).
    // The end-to-end sensitivity is gains * ISO * (G_agg * P / N^2 * C^2 * t * QE); for the green channel that
    // is 1.0 * 90 * 1.071e-2 = 0.96, so the brightest scene content still clips to white. Do NOT drop
    // ISO to "under-expose": the exposure math runs on the ALREADY gamma-encoded render (phys_cam_raygen.cu
    // applies pow(1/gamma) before the filter chain), so halving ISO caps the brightest possible pixel at 54%
    // grey and crushes the whole frame into the bottom half of the range -- washed out, not under-exposed.
    // The low-light character here comes from the cos^4 vignette and the sensor grain instead.
    cam->SetCtrlParameters(4.0f, 0.256f, 90.0f, focal_length, (float)radius);
    cam->SetModelParameters(sensor_width, pixel_size, max_scene_light_amount, rgb_QE_vec, gain_params, noise_params);
    cam->SetName("physcam_grain");
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
    manager->AddSensor(cam);

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        printf("could not create %s: %s\n", out_dir.c_str(), ec.message().c_str());
        return 1;
    }

    printf("Physics-based sensor showcase, ChPhysCameraSensor. PNGs -> %s\n", out_dir.c_str());
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
        cam->SetOffsetPose(orbitPose(CH_2PI * f / nframes));  // full orbit over 150 frames
        manager->Update();
        if (auto buf = cam->GetMostRecentBuffer<UserRGBA16BufferPtr>())
            if (buf->Buffer)
                save_rgba16_png(out_dir + "frame_" + std::to_string(saved++) + ".png", buf);
        time += step;
    }
    printf("wrote %d frames\n", saved);
    return 0;
}
