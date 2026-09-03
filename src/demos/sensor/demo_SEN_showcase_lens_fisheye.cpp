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
// SHOWCASE: wide-angle FISHEYE lens.
// A stationary Audi on flat terrain under the shipped sky_2_4k HDR, camera orbiting the car with a very
// wide FOV_LENS (equidistant fisheye) lens model so the horizon and env map bend around the frame.
// HEADLESS: writes 150 PNGs to SENSOR_OUTPUT/SHOWCASE_LENS_FISHEYE/ then returns (no live window).
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

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_LENS_FISHEYE/";

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
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    // Orbit params (see loop). Initial offset pose looking at the car.
    const double radius = 6.0, height = 1.8;
    const ChVector3d look(0, 0, 0.8);
    auto orbitPose = [&](double ang) {
        ChVector3d off(radius * std::cos(ang), radius * std::sin(ang), height);
        ChVector3d d = (look - off).GetNormalized();
        return ChFrame<double>(off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    };

    // Wide-angle FISHEYE: FOV_LENS lens model with a very wide horizontal FOV (~2.5 rad).
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 500.0f, orbitPose(0.0), 1280, 720, 2.5f /*hFOV ~143 deg fisheye*/, 1 /*ss*/,
                                                         CameraLensModelType::FOV_LENS, false /*GI*/, false /*denoiser*/);
    cam->SetName("lens_fisheye");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + ""));
    manager->AddSensor(cam);

    printf("Fisheye (FOV_LENS) showcase. PNGs -> SENSOR_OUTPUT/SHOWCASE_LENS_FISHEYE/\n");
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
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
        time += step;
    }
    return 0;
}
