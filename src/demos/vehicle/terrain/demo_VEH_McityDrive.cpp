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
//
// Drive an Audi around the Mcity digital twin, under keyboard control.
//
// What this needs
// ---------------
// Chrono::Vehicle and Chrono::VSG, and nothing else. There is no road network
// here, no OpenDRIVE, no esmini: the scene is a set of meshes with placements,
// and the ground is those same meshes. Chrono::Scenario adds lane semantics,
// routing and OpenSCENARIO playback on top, which a self-driven car does not
// need -- see demo_SCEN_mcity for the version that routes itself.
//
// The two pieces that make it work
// --------------------------------
// ChSceneryModel loads a placement manifest: a few hundred meshes instanced a
// few thousand times, one visual shape per mesh however often it appears.
//
// ChMeshTerrain then answers tire queries out of those same triangles, through
// a ChMeshHeightField. That matters more than it sounds. The alternative is a
// separately authored surface, and on Mcity the OpenDRIVE elevation profile and
// the artist's road mesh disagree by -0.24 to +0.29 m at the 5th and 95th
// percentiles -- enough to watch the car float and sink as it drives. Reading
// the drawn geometry makes the two the same surface by construction, and costs
// no solver time: it is a grid lookup, not a collision shape.
//
// Getting the scene
// -----------------
// Chrono ships the converter, not the scene -- the Mcity assets are a separate
// dataset under their own licence. Build it once:
//
//   src/demos/scenario/mcity/setup_mcity.sh --repo /path/to/mcity-digital-twin
//
// or let it fetch over HTTPS by omitting --repo. Add --foliage for vegetation.
//
// Controls: W/S throttle and brake, A/D steer, and the usual VSG camera keys.
//
// =============================================================================

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/ChMeshTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChSceneryModel.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

// A pose on a real Mcity lane, taken once from the OpenDRIVE network so that this demo does not
// have to read it. Road 29, right-hand lane, a few metres in, facing along the carriageway.
const ChVector3d kStartPos(158.923, 62.991, 274.6);
const double kStartYaw = 1.285431;

// Vegetation levels, as produced by mcity/build_configs.sh. "none" is always available; the rest
// require setup_mcity.sh --foliage.
std::string ManifestFor(const std::string& level) {
    if (level == "none") return "mcity_scene.json";
    if (level == "trees") return "mcity_scene_trees_bare.json";
    if (level == "trees-leaf") return "mcity_scene_trees_leaf.json";
    if (level == "shrubs") return "mcity_scene_all_bare.json";
    if (level == "full") return "mcity_scene_full.json";
    return "";
}

int main(int argc, char** argv) {
    std::string foliage = "none";
    std::string data_dir;
    std::string terrain_kind = "mesh";  // mesh | rigid
    double radius = 90.0;
    double max_seconds = 0;  // 0 = until the window is closed
    bool save_frames = false;
    std::string ground_obj_override;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--foliage" && i + 1 < argc)
            foliage = argv[++i];
        else if (a == "--data" && i + 1 < argc)
            data_dir = argv[++i];
        else if (a == "--radius" && i + 1 < argc)
            radius = std::atof(argv[++i]);
        else if (a == "--terrain" && i + 1 < argc)
            terrain_kind = argv[++i];
        else if (a == "--seconds" && i + 1 < argc)
            max_seconds = std::atof(argv[++i]);
        else if (a == "--save-frames")
            save_frames = true;
        else if (a == "--ground-obj" && i + 1 < argc)
            ground_obj_override = argv[++i];
        else {
            printf("usage: demo_VEH_McityDrive [--foliage none|trees|trees-leaf|shrubs|full]\n");
            printf("                           [--terrain mesh|rigid] [--radius M] [--data DIR]\n");
            return 1;
        }
    }
    if (ManifestFor(foliage).empty()) {
        printf("unknown --foliage level: %s\n", foliage.c_str());
        return 1;
    }

    std::string mcity_dir = data_dir.empty() ? GetChronoDataFile("mcity") : data_dir;
    std::string manifest = mcity_dir + "/" + ManifestFor(foliage);

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ---------------------------------------------------------------------------------------
    // The scene, and the ground under it
    // ---------------------------------------------------------------------------------------

    ChSceneryModel scenery;
    ChSceneryOptions sopt;
    sopt.tile_size = 40.0;  // the unit the visual system switches on and off by distance

    // Everything is ground unless it obviously is not. A whitelist is the tempting choice and the
    // wrong one: it silently omits any surface nobody remembered to name, and the failure mode is
    // a vehicle sinking through geometry it can see. Overhead objects are harmless because the
    // height query is z-aware, so these exclusions are only about build cost.
    sopt.ground_all = true;
    sopt.ground_exclude_groups = {"Foliage_Instanced", "TrafficPoles", "StreetLights",
                                  "TrafficLights", "TrafficLightCables"};
    sopt.ground_exclude = {"Pole", "Sign", "TrafficLight", "Cable", "Fence", "GuardRail",
                           "Facade", "Building", "StreetLight", "Hydrant", "Bollard", "Barrier",
                           "Container", "WaterTower", "Pavilion", "Basketball", "Dumpster",
                           "BusStop", "Bench", "Chair", "Table", "Meter", "Charger", "Barrel",
                           "Rock", "Camera", "GPSBlocker", "MailBox", "TrashCan", "BikeRack"};

    if (!scenery.Load(sys, manifest, sopt)) {
        printf("Could not read %s\n", manifest.c_str());
        printf("Build the scene first:\n");
        printf("  src/demos/scenario/mcity/setup_mcity.sh --repo /path/to/mcity-digital-twin\n");
        return 1;
    }
    scenery.ReportTo(std::cout);

    auto field = scenery.GetGroundHeightField();
    if (!field || !field->IsReady()) {
        printf("No ground could be built from %s\n", manifest.c_str());
        return 1;
    }
    // Two ways to stand on the same triangles.
    //
    //   mesh   ChMeshTerrain: a grid lookup, no bodies and no collision system. Cheapest, and
    //          enough for handling tires, which reach the ground only through GetHeight,
    //          GetNormal and GetPoint. Nothing can collide with it.
    //   rigid  RigidTerrain over a merged ground mesh: the standard route, and the only one
    //          that gives a solid world, so rigid and FEA tires work and other bodies can hit
    //          it. Measured slightly faster per step than the height field, since a BVH raycast
    //          beats a grid bucket here. Costs a 227k-triangle collision mesh and the one-time
    //          export of that mesh.
    //
    //          One ordering trap: RigidTerrain answers height queries by raycasting the
    //          collision system, which does not exist until the first DoStepDynamics. Querying
    //          it during setup returns its miss value of 0 -- see the spawn height below.
    std::unique_ptr<ChMeshTerrain> mesh_terrain;
    std::unique_ptr<RigidTerrain> rigid_terrain;
    ChTerrain* terrain_ptr = nullptr;

    if (terrain_kind == "rigid") {
        std::string ground_obj = ground_obj_override.empty() ? mcity_dir + "/mcity_ground.obj"
                                                             : ground_obj_override;
        std::ifstream have(ground_obj);
        if (!have.good()) {
            printf("  writing merged ground mesh to %s\n", ground_obj.c_str());
            if (!scenery.WriteGroundMesh(ground_obj)) {
                printf("  could not write %s\n", ground_obj.c_str());
                return 1;
            }
        }
        auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
        mat->SetFriction(0.9f);
        mat->SetRestitution(0.01f);
        rigid_terrain = std::make_unique<RigidTerrain>(&sys);
        // visualization off: the scenery already draws these triangles, and drawing them twice
        // would z-fight.
        rigid_terrain->AddPatch(mat, CSYSNORM, ground_obj, true, 0, false);
        rigid_terrain->Initialize();
        terrain_ptr = rigid_terrain.get();
    } else {
        mesh_terrain = std::make_unique<ChMeshTerrain>(field, 0.9f);
        terrain_ptr = mesh_terrain.get();
    }
    ChTerrain& terrain = *terrain_ptr;

    // ---------------------------------------------------------------------------------------
    // The vehicle
    // ---------------------------------------------------------------------------------------

    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));

    // Ask the height field, not the terrain, for the spawn height.
    //
    // RigidTerrain answers by raycasting the collision system, which has not been built before
    // the first DoStepDynamics -- so the ray misses and RigidTerrain::GetHeight returns its
    // miss value of 0. The car then spawns 274 m below the site, resting on nothing, and the
    // chase camera follows it into empty sky. The height field is populated at load and has no
    // such ordering constraint.
    double ground_z = kStartPos.z();
    {
        double z;
        if (field->HeightBelow(kStartPos.x(), kStartPos.y(), kStartPos.z() + 5.0, z))
            ground_z = z;
    }
    ChCoordsys<> start(ChVector3d(kStartPos.x(), kStartPos.y(), ground_z + 0.5),
                       QuatFromAngleZ(kStartYaw));
    audi.Initialize(start);
    audi.GetChassis()->SetFixed(false);
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::MESH);
    audi.SetSteeringVisualizationType(VisualizationType::MESH);
    audi.SetWheelVisualizationType(VisualizationType::MESH);

    auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
    auto transmission =
        ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
    audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));

    for (auto& axle : audi.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_Pac02Tire.json"));
            audi.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Keyboard control and rendering
    // ---------------------------------------------------------------------------------------

    ChInteractiveDriver driver(audi);
    driver.SetSteeringDelta(0.04);
    driver.SetThrottleDelta(0.08);
    driver.SetBrakingDelta(0.3);
    driver.Initialize();

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("Mcity");
    vis->SetWindowSize(1600, 900);
    vis->AttachVehicle(&audi);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 7.0, 0.6);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->AttachDriver(&driver);
    vis->Initialize();

    // Draw only what is near the car. A town-sized static scene is mostly irrelevant at any
    // instant, and gating on distance makes the frame cost follow what is nearby rather than how
    // large the site is. 0 disables it.
    if (radius > 0)
        vis->EnableVisualStreaming(true, radius, 20.0);

    setvbuf(stdout, nullptr, _IOLBF, 0);
    printf("\nW/S throttle and brake, A/D steer.\n\n");
    auto t_start = std::chrono::steady_clock::now();
    double next_report = 0;
    int frame_idx = 0;

    const double step = 1e-3;
    const double render_step = 1.0 / 50;
    double next_render = 0;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        double time = sys.GetChTime();
        if (max_seconds > 0 && time >= max_seconds)
            break;
        if (time >= next_report) {
            double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
            printf("t=%6.2f s  wall=%6.2f s  (%.3fx real time)\n", time, wall, wall > 0 ? time / wall : 0);
            next_report += 1.0;
        }

        if (time >= next_render) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            if (save_frames) {
                char fn[256];
                snprintf(fn, sizeof(fn), "mcity_out/frame_%04d.png", frame_idx++);
                vis->WriteImageToFile(fn);
            }
            next_render += render_step;
        }
        if (radius > 0)
            vis->SetStreamingFocus(audi.GetPos());

        DriverInputs inputs = driver.GetInputs();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        audi.Synchronize(time, inputs, terrain);
        vis->Synchronize(time, inputs);

        driver.Advance(step);
        terrain.Advance(step);
        audi.Advance(step);
        vis->Advance(step);
        sys.DoStepDynamics(step);

        realtime.Spin(step);
    }

    return 0;
}
