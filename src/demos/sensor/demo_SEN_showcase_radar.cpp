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
// SHOWCASE (headless): a forward-looking Doppler RADAR.
// The Audi drives toward a field of obstacles; a front-mounted radar returns range + azimuth + Doppler for
// each hit. Each frame's returns (ChFilterRadarAccess) are dumped to a small binary; the webp step renders a
// top-down radar plot -- returns placed in the sensor's forward FOV wedge, coloured by closing speed (Doppler).
//   -> SENSOR_OUTPUT/SHOWCASE_RADAR/{scan,cam}/   (custom return buffers, rendered to webp in python)
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_RADAR/";

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
                tire->SetStepsize(1e-4);  // stable suspension (matches `autonomous`)
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    // a field of obstacles ahead (+x) that the driving Audi closes on -> non-zero Doppler
    auto addBox = [&](ChVector3d sz, ChVector3d pos) {
        auto b = chrono_types::make_shared<ChBodyEasyBox>(sz.x(), sz.y(), sz.z(), 1000, true, false);
        b->SetFixed(true);
        b->SetPos(pos);
        sys.Add(b);
    };
    auto addSphere = [&](double r, ChVector3d pos) {
        auto b = chrono_types::make_shared<ChBodyEasySphere>(r, 1000, true, false);
        b->SetFixed(true);
        b->SetPos(pos);
        sys.Add(b);
    };
    addBox({1.4, 1.4, 1.8}, {18, 4.0, 0.9});
    addBox({1.4, 1.4, 1.8}, {26, -5.0, 0.9});
    addSphere(1.0, {30, 3.2, 1.0});
    addBox({1.2, 1.2, 1.6}, {40, -3.2, 0.8});
    addSphere(1.1, {48, 5.0, 1.1});
    addBox({1.4, 1.4, 2.0}, {52, -4.0, 1.0});

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    // front-mounted forward-looking radar
    const unsigned RW = 250, RH = 100;
    const float HFOV = (float)(CH_PI / 2), VFOV = (float)(CH_PI / 6), MAXD = 100.0f;
    // mounted just AHEAD of the front bumper (x=3.0) with a 0.6 m near-clip, so beams clear the car's own
    // body instead of self-hitting it at ~0.2 m.
    ChFrame<double> radar_off(ChVector3d(3.0, 0, 0.7), QUNIT);
    auto radar = chrono_types::make_shared<ChRadarSensor>(audi.GetChassisBody(), 10.0f, radar_off, RW, RH, HFOV, VFOV, MAXD, 0.6f /*clip_near*/);
    radar->SetName("showcase_radar");
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());  // expose the raw returns
    manager->AddSensor(radar);

    // COMPANION chase camera at the SAME update rate, so every scan has a matching RGB frame. The webp step
    // pairs them side-by-side -- without it you cannot tell what the range plot is actually looking at.
    ChVector3d cam_off(-9.5, -5.0, 4.2);
    ChVector3d cam_look(9.0, 0.0, 0.6);
    ChVector3d cd = (cam_look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(cd.y(), cd.x())) * QuatFromAngleY(-std::asin(cd.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 10.0f, cam_pose, 720, 720, (float)(CH_PI / 3), 1, CameraLensModelType::PINHOLE, false, false);
    cam->SetName("radar_companion_cam");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "cam/"));
    manager->AddSensor(cam);

    printf(
        "Showcase radar (headless): forward Doppler radar -> top-down plot.\n"
        "  -> SENSOR_OUTPUT/SHOWCASE_RADAR/{scan,cam}/\n");
    std::filesystem::create_directories(out_dir + "scan");
    const double step = 1e-3;
    double time = 0, last_ts = -1;
    int saved = 0;
    while (time < 6.0) {
        DriverInputs in;
        in.m_throttle = 0.5;
        in.m_steering = 0.0;  // straight down the lane: a sine input integrates into a net heading change,
                              // which curved the car off-course and into the obstacles in.m_braking = 0.0;
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();

        auto buf = radar->GetMostRecentBuffer<UserRadarBufferPtr>();
        if (buf && buf->Buffer && buf->TimeStamp > last_ts + 1e-6) {
            last_ts = buf->TimeStamp;
            const std::string fn = out_dir + "scan/frame_" + std::to_string(saved++) + ".bin";
            std::ofstream os(fn, std::ios::binary);
            int32_t w = (int32_t)buf->Width, h = (int32_t)buf->Height;
            float hdr[3] = {HFOV, VFOV, MAXD};
            os.write((char*)&w, 4);
            os.write((char*)&h, 4);
            os.write((char*)hdr, sizeof(hdr));
            // sensor->world rotation (row-major 3x3). doppler_velocity is a WORLD-frame relative velocity
            // while azimuth/elevation are sensor-frame, so the plot needs this to project the velocity onto
            // each beam and recover the true RADIAL (closing) Doppler speed.
            ChFrame<double> sf = audi.GetChassisBody()->GetFrameRefToAbs() * radar_off;
            ChMatrix33<double> R = sf.GetRotMat();
            float Rf[9];
            for (int rr = 0; rr < 3; ++rr)
                for (int cc = 0; cc < 3; ++cc)
                    Rf[rr * 3 + cc] = (float)R(rr, cc);
            os.write((char*)Rf, sizeof(Rf));
            size_t n = (size_t)w * h;
            for (size_t i = 0; i < n; ++i) {
                const RadarReturn& r = buf->Buffer[i];
                float rec[7] = {r.range, r.azimuth, r.elevation, r.doppler_velocity[0], r.doppler_velocity[1], r.doppler_velocity[2], r.amplitude};
                os.write((char*)rec, sizeof(rec));
            }
        }
        time += step;
    }
    printf("  saved %d radar frames\n", saved);
    return 0;
}
