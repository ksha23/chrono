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
// SHOWCASE (headless): a real 360-degree LIDAR.
// A roof-mounted multi-beam lidar scans the scene as the Audi drives past a lane of obstacles. Each scan's
// range/intensity buffer (ChFilterDIAccess) is dumped to a small binary; the webp step reconstructs a
// car-centric BIRD'S-EYE point cloud (points coloured by height) -- the classic self-driving lidar view.
//   -> SENSOR_OUTPUT/SHOWCASE_LIDAR/{scan,cam}/   (custom range buffers, rendered to webp in python)
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
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_LIDAR/";

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

    // obstacle lane + two low side walls -> gives the lidar BEV clear structure to scan
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
    addBox({1.2, 1.2, 1.6}, {12, 3.0, 0.8});
    addSphere(0.9, {20, -3.0, 0.9});
    addBox({0.9, 0.9, 2.0}, {28, 3.5, 1.0});
    addBox({1.0, 2.5, 1.2}, {36, -3.5, 0.6});
    addSphere(0.7, {44, 2.5, 0.7});
    addBox({40.0, 0.4, 1.4}, {24, 9.0, 0.7});   // left wall
    addBox({40.0, 0.4, 1.4}, {24, -9.0, 0.7});  // right wall

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    // roof-mounted 360-degree multi-beam lidar
    const unsigned LW = 1000, LH = 64;
    const float HFOV = (float)(2 * CH_PI), VMAX = 0.26f, VMIN = -0.26f, MAXD = 60.0f;
    auto lidar = chrono_types::make_shared<ChLidarSensor>(audi.GetChassisBody(), 10.0f, ChFrame<double>(ChVector3d(0, 0, 1.9), QUNIT), LW, LH, HFOV, VMAX, VMIN, MAXD);
    lidar->SetName("showcase_lidar");
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());  // expose the range/intensity buffer
    manager->AddSensor(lidar);

    // COMPANION chase camera at the SAME update rate, so every scan has a matching RGB frame. The webp step
    // pairs them side-by-side -- without it you cannot tell what the range plot is actually looking at.
    ChVector3d cam_off(-9.5, -5.0, 4.2);
    ChVector3d cam_look(9.0, 0.0, 0.6);
    ChVector3d cd = (cam_look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(cd.y(), cd.x())) * QuatFromAngleY(-std::asin(cd.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 10.0f, cam_pose, 720, 720, (float)(CH_PI / 3), 1, CameraLensModelType::PINHOLE, false, false);
    cam->SetName("lidar_companion_cam");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "cam/"));
    manager->AddSensor(cam);

    printf(
        "Showcase lidar (headless): 360-degree scan -> BEV point cloud.\n"
        "  -> SENSOR_OUTPUT/SHOWCASE_LIDAR/{scan,cam}/\n");
    std::filesystem::create_directories(out_dir + "scan");
    const double step = 1e-3;
    double time = 0, last_ts = -1;
    int saved = 0;
    while (time < 6.0) {
        DriverInputs in;
        in.m_throttle = 0.4;
        in.m_steering = 0.0;  // straight down the lane: a sine input integrates into a net heading change,
                              // which curved the car off-course and into the obstacles in.m_braking = 0.0;
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();

        auto buf = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
        if (buf && buf->Buffer && buf->TimeStamp > last_ts + 1e-6) {
            last_ts = buf->TimeStamp;
            const std::string fn = out_dir + "scan/frame_" + std::to_string(saved++) + ".bin";
            std::ofstream os(fn, std::ios::binary);
            int32_t w = (int32_t)buf->Width, h = (int32_t)buf->Height;
            float hdr[4] = {HFOV, VMAX, VMIN, MAXD};
            os.write((char*)&w, 4);
            os.write((char*)&h, 4);
            os.write((char*)hdr, sizeof(hdr));
            size_t n = (size_t)w * h;
            for (size_t i = 0; i < n; ++i) {
                float r = buf->Buffer[i].range;
                os.write((char*)&r, 4);
            }
            for (size_t i = 0; i < n; ++i) {
                float in_ = buf->Buffer[i].intensity;
                os.write((char*)&in_, 4);
            }
        }
        time += step;
    }
    printf("  saved %d lidar scans\n", saved);
    return 0;
}
