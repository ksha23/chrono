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
// SHOWCASE: NIGHT DRIVE with SPOT-LIGHT HEADLIGHTS.
// Near-black night scene (no sun, no env map, tiny ambient) lit only by two forward-pointing headlight spot
// cones at the front of the Audi. Camera orbits across the front to sweep through the twin beams.
// Headless: 150 PNG frames saved to SENSOR_OUTPUT/SHOWCASE_NIGHT_HEADLIGHTS/. No live window.
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

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_NIGHT_HEADLIGHTS/";

int main(int argc, char** argv) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    // flat terrain patch (portable) -- the beams pool on the ground ahead of the car
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
    // night: no sun, no env map, only a whisper of blue ambient + near-black sky. Kept very low so the
    // gamma curve doesn't lift it into "dusk" -- the headlight pools should be the only bright thing.
    manager->scene->SetAmbientLight(ChVector3f(0.008f, 0.008f, 0.014f));
    manager->scene->SetBackground(Background{BackgroundMode::SOLID_COLOR, ChVector3f(0.01f, 0.01f, 0.03f), ChVector3f(), ""});

    // STATIC camera behind & above the parked car, looking forward down the road. The car sits dark; part-
    // way through, the headlights switch ON and two beams light up the road ahead (seen from behind).
    ChVector3d cam_off(-8.5, 3.4, 3.1);  // behind, up, and to the left -> rear 3/4 view down the road
    ChVector3d look(14.0, -0.4, 0.0);    // far ahead at ground level, where the beams land
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 500.0f, cam_pose, 1280, 720, (float)(CH_PI / 3), 2, CameraLensModelType::PINHOLE, false /*GI*/,
                                                         true /*denoiser*/);
    cam->SetName("showcase_night_headlights");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + ""));
    manager->AddSensor(cam);

    printf("Showcase night headlights. PNGs -> SENSOR_OUTPUT/SHOWCASE_NIGHT_HEADLIGHTS/. 150 frames...\n");
    const double step = 2e-3;
    const int nframes = 150;
    const int on_frame = 42;  // headlights switch on here (a beat of darkness first)
    const int ramp = 16;      // frames over which they brighten to full
    double time = 0;
    auto body = audi.GetChassisBody();
    DriverInputs in;
    in.m_throttle = 0;
    in.m_steering = 0;
    in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        // IMPORTANT: the two headlight spots are added EVERY frame, even while "off" (intensity 0). If the
        // scene's light list is left empty the renderer may synthesize a default key + fill rig, which
        // would light the whole scene brightly during the off phase and then snap dark once the beams appear.
        float k = (f < on_frame) ? 0.f : std::min(1.f, (float)(f - on_frame + 1) / (float)ramp);  // 0->1 ramp
        // two headlight spots at the front corners, aimed forward and only ~15 deg down so the beams
        // reach far down the road (long, clearly-visible footprints seen from behind the car).
        ChVector3d fwd = body->TransformDirectionLocalToParent(ChVector3d(1, 0, 0));
        ChVector3d rgt = body->TransformDirectionLocalToParent(ChVector3d(0, -1, 0));
        ChVector3d base = body->GetPos() + fwd * 1.9 + ChVector3d(0, 0, 0.62);
        ChVector3d bd = (fwd * 0.965 - ChVector3d(0, 0, 0.26)).GetNormalized();  // ~15 deg down
        ChVector3f bdir((float)bd.x(), (float)bd.y(), (float)bd.z());
        ChVector3d L = base - rgt * 0.75;
        ChVector3d R = base + rgt * 0.75;
        ChColor beam(12.0f * k, 11.0f * k, 9.0f * k);
        manager->scene->ClearLights();
        // ChOptixScene::AddSpotLight(pos, color, max_range, light_dir, angle_falloff_start, angle_range,
        //                       const_color).
        // Angles are FULL cone angles in radians: 32 deg cone with a 20 deg soft edge.
        // const_color must be false to get that soft edge: const_color = true (the default) means
        // "no attenuation", which in ChOptixSpotLight skips the angular falloff as well as the
        // distance falloff, leaving a flat beam with a hard cone cutoff.
        manager->scene->AddSpotLight(ChVector3f((float)L.x(), (float)L.y(), (float)L.z()), beam, 60.f, bdir, 0.20944f, 0.55851f, false);
        manager->scene->AddSpotLight(ChVector3f((float)R.x(), (float)R.y(), (float)R.z()), beam, 60.f, bdir, 0.20944f, 0.55851f, false);

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
