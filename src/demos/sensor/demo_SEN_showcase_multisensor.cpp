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
// SHOWCASE (headless): a genuine MULTI-SENSOR rig -- FOUR co-located sensors run at once off
// the same driving Audi and each produces a capturable image, tiled into a 2x2 panel by the webp step:
//   * RGB camera            (ChCameraSensor)        -> multisensor/rgb/
//   * Depth camera          (ChDepthCamera)         -> multisensor/depth/
//   * Surface-normal camera (ChNormalCamera)        -> multisensor/normal/
//   * Segmentation camera   (ChSegmentationCamera)  -> multisensor/seg/
// The Audi drives forward past a lane of colored, semantically-labelled obstacles on flat terrain under the
// shipped sky_2_4k HDR. All four sensors share one chase pose so the four panels are perfectly registered.
//
// Vehicle config mirrors the stable `autonomous` demo: 1e-3 sim step + 1e-4 tire substep. The coarser step
// this demo used before let the suspension buckle repeatedly; the fine tire substep fixes it.
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

// Give every visual material on a body a semantic (class, instance) label so the segmentation camera can
// separate them. Walks all shapes; adds a material carrying the id if a shape has none.
static void SetSemantic(std::shared_ptr<ChBody> body, unsigned short cls, unsigned short inst) {
    auto vm = body->GetVisualModel();
    if (!vm)
        return;
    for (auto& si : vm->GetShapeInstances()) {
        auto& mats = si.shape->GetMaterials();
        if (mats.empty()) {
            auto m = chrono_types::make_shared<ChVisualMaterial>();
            m->SetClassID(cls);
            m->SetInstanceID(inst);
            si.shape->AddMaterial(m);
        } else {
            for (auto& m : mats) {
                m->SetClassID(cls);
                m->SetInstanceID(inst);
            }
        }
    }
}

const std::string out_dir = "SENSOR_OUTPUT/SHOWCASE_MULTISENSOR/";

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

    // Audi, starting at origin, facing +x -- STABLE config (fine tire substep, like the autonomous demo)
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
                tire->SetStepsize(1e-4);  // fine tire substep -> stable suspension (matches `autonomous`)
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }
    SetSemantic(audi.GetChassisBody(), 2, 1);  // car = class 2

    // a lane of colored obstacles ahead (+x) that the Audi drives past, each a distinct semantic class
    unsigned short next_class = 10;
    auto addObstacle = [&](std::shared_ptr<ChBody> body, ChVector3d pos, ChColor c) {
        body->SetFixed(true);
        body->SetPos(pos);
        auto m = chrono_types::make_shared<ChVisualMaterial>();
        m->SetDiffuseColor(c);
        m->SetClassID(next_class);
        m->SetInstanceID(next_class);  // one class per obstacle
        body->GetVisualModel()->GetShapeInstances()[0].shape->AddMaterial(m);
        sys.Add(body);
        ++next_class;
    };
    addObstacle(chrono_types::make_shared<ChBodyEasyBox>(1.2, 1.2, 1.5, 1000, true, false), ChVector3d(12.0, 3.0, 0.8), ChColor(0.85f, 0.18f, 0.18f));
    addObstacle(chrono_types::make_shared<ChBodyEasySphere>(0.9, 1000, true, false), ChVector3d(20.0, -3.0, 0.9), ChColor(0.18f, 0.75f, 0.28f));
    addObstacle(chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, 0.6, 2.0, 1000, true, false), ChVector3d(28.0, 3.5, 1.0), ChColor(0.2f, 0.45f, 0.95f));
    addObstacle(chrono_types::make_shared<ChBodyEasyBox>(1.0, 2.2, 1.2, 1000, true, false), ChVector3d(36.0, -3.5, 0.6), ChColor(0.9f, 0.8f, 0.2f));
    addObstacle(chrono_types::make_shared<ChBodyEasySphere>(0.7, 1000, true, false), ChVector3d(44.0, 2.5, 0.7), ChColor(0.85f, 0.35f, 0.85f));

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.02625f, 0.50710f);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    // one shared chase pose; each sensor is 640x360 so a 2x2 tile is exactly 1280x720
    const unsigned W = 640, H = 360;
    const float FOV = (float)(CH_PI / 3);
    ChVector3d cam_off(-8.0, -3.5, 3.0);
    ChVector3d look(2.0, 0.0, 0.7);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto parent = audi.GetChassisBody();

    auto rgb = chrono_types::make_shared<ChCameraSensor>(parent, 30.0f, pose, W, H, FOV, 1, CameraLensModelType::PINHOLE, false, false);
    rgb->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "rgb/"));
    manager->AddSensor(rgb);

    auto depth = chrono_types::make_shared<ChDepthCamera>(parent, 30.0f, pose, W, H, FOV, 45.0f);
    depth->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "depth/"));
    manager->AddSensor(depth);

    auto normal = chrono_types::make_shared<ChNormalCamera>(parent, 30.0f, pose, W, H, FOV);
    normal->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "normal/"));
    manager->AddSensor(normal);

    auto seg = chrono_types::make_shared<ChSegmentationCamera>(parent, 30.0f, pose, W, H, FOV);
    seg->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "seg/"));
    manager->AddSensor(seg);

    printf(
        "Showcase multisensor (headless): RGB + Depth + Normal + Segmentation.\n"
        "  -> SENSOR_OUTPUT/SHOWCASE_MULTISENSOR/{rgb,depth,normal,seg}/\n");
    const double step = 1e-3;  // fine sim step (stable), matches `autonomous`
    double time = 0;
    // 5.0 s @ 30 Hz => ~150 saved frames per sensor.
    while (time < 5.0) {
        DriverInputs in;
        in.m_throttle = 0.4;
        // Straight down the lane. A sine steering input integrates into a net heading change, so the
        // car drifted off-course and clipped the obstacles instead of driving cleanly past them.
        in.m_steering = 0.0;
        in.m_braking = 0.0;
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
