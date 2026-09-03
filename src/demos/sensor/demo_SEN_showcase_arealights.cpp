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
// SHOWCASE: AREA LIGHTS with SOFT SHADOWS.
// A dark studio scene (no env map, near-zero ambient) lit only by an overhead disk light and a side-fill
// rectangle light. The finite light area casts soft, penumbra-edged shadows of the Audi onto the ground.
// Headless: slow orbit, 150 PNG frames saved to SENSOR_OUTPUT/SHOWCASE_AREALIGHTS/. No live window.
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_AREALIGHTS/";

int main(int argc, char** argv) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    // flat terrain patch (portable) -- catches the soft shadows
    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

    // Audi, at origin, facing +x
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
    // dark studio: no env map, near-zero ambient, black background -> only the area lights illuminate.
    manager->scene->SetAmbientLight(ChVector3f(0.02f, 0.02f, 0.03f));
    manager->scene->SetBackground(Background{BackgroundMode::SOLID_COLOR, ChVector3f(0.01f, 0.01f, 0.015f), ChVector3f(), ""});
    // overhead disk key light (5 m up, normal pointing straight down), 1.2 m radius -> broad soft shadow
    manager->scene->AddDiskLight(ChVector3f(0.5f, 0.f, 5.0f), ChColor(9.0f, 8.7f, 8.2f), 60.f, ChVector3f(0.f, 0.f, -1.f), 1.2f);
    // side-fill rectangle light (to the car's left, ~2.5 m up), 3 m long x 1.5 m tall
    manager->scene->AddRectangleLight(ChVector3f(-1.0f, 4.5f, 2.5f), ChColor(3.5f, 3.6f, 4.2f), 60.f, ChVector3f(3.0f, 0.f, 0.f), ChVector3f(0.f, 0.f, 1.5f));

    ChVector3d look(0, 0, 0.8);
    ChVector3d cam_off(6.0, 0.0, 1.8);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    // Area-light soft shadows are stochastic (each sample jitters the light-surface point), so they need
    // many samples/pixel to smooth out -> ss=4 (16 samples) + denoiser kills the penumbra grain.
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 500.0f, cam_pose, 1280, 720, (float)(CH_PI / 3), 4, CameraLensModelType::PINHOLE, false /*GI*/,
                                                         true /*denoiser*/);
    cam->SetName("showcase_arealights");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + ""));
    manager->AddSensor(cam);

    printf("Showcase area lights. PNGs -> SENSOR_OUTPUT/SHOWCASE_AREALIGHTS/. 150 frames...\n");
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
    DriverInputs in;
    in.m_throttle = 0;
    in.m_steering = 0;
    in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        double ang = 2.0 * CH_PI * (double)f / (double)nframes;  // full orbit (synced with the other demos)
        cam_off = ChVector3d(6.0 * std::cos(ang), 6.0 * std::sin(ang), 1.8);
        d = (look - cam_off).GetNormalized();
        cam->SetOffsetPose(ChFrame<double>(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z()))));

        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();
        time += step;
    }
    return 0;
}
