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
// SHOWCASE: GLOSSY HDR ENVIRONMENT REFLECTIONS.
// The Audi sits under the driving_school_4k HDR environment map, which lights the scene AND is mirrored in
// the car's glossy paint and glass. As the camera orbits, the reflected surroundings sweep across the body.
// Headless: full orbit, 150 PNG frames saved to SENSOR_OUTPUT/SHOWCASE_ENVMAP/. No live window.
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

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_ENVMAP/";

int main(int argc, char** argv) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    // flat terrain patch (portable)
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
    // Striking HDR environment: lights the scene and provides the reflected surroundings on the paint.
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/driving_school_4k.hdr"), 1.2f);
    manager->scene->SetAmbientLight(ChVector3f(0.20f, 0.20f, 0.22f));

    ChVector3d look(0, 0, 0.8);
    ChVector3d cam_off(6.0, 0.0, 1.8);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 500.0f, cam_pose, 1280, 720, (float)(CH_PI / 3), 2, CameraLensModelType::PINHOLE, false /*GI*/,
                                                         false /*denoiser*/);
    cam->SetName("showcase_envmap");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + ""));
    manager->AddSensor(cam);

    printf("Showcase env-map reflections. PNGs -> SENSOR_OUTPUT/SHOWCASE_ENVMAP/. 150 frames...\n");
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
    DriverInputs in;
    in.m_throttle = 0;
    in.m_steering = 0;
    in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        double ang = 2.0 * CH_PI * (double)f / (double)nframes;  // full orbit -> reflections sweep across
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
