// An ASAM OpenSCENARIO scenario run in co-simulation with Chrono.
//
// esmini owns the story and the ambient traffic; Chrono owns the ego vehicle's dynamics. Each
// step the Chrono ego state is reported into the scenario, so the scenario's triggers -- time
// headway, relative distance -- evaluate against real multibody dynamics rather than against a
// kinematic stand-in. That is the whole point of the coupling: in cut-in_external.xosc the
// OverTaker's lane change fires on a TimeHeadwayCondition measured against the ego, so how hard
// the Audi actually accelerates decides when the cut-in happens.
//
// Usage:  ./scenario_drive [path/to/scenario.xosc] [max_seconds] [ego_speed_mps]
//
// Defaults to esmini's cut-in_external.xosc, which declares an externalController on the ego
// precisely so that an outside simulator can drive it. Override the esmini resource root with
// ESMINI_ROOT.
//
// On the ego's speed: when a scenario assigns an externalController to the ego, esmini does not
// apply the ego's Init SpeedAction -- the whole point of external control is that the outside
// simulator decides how the ego moves. So the AbsoluteTargetSpeed in the .xosc is advisory, and
// querying the ego's state right after loading returns 0 rather than the declared speed. The
// speed therefore has to be supplied here. cut-in_external.xosc declares 30 m/s, which is the
// default below.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_scenario/ChOpenDriveNetwork.h"
#include "chrono_scenario/ChOpenDriveTerrain.h"
#include "chrono_scenario/ChScenarioPlayer.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/metal/ChFilterMetalVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::scenario;
using namespace chrono::sensor;

namespace {
const char* kChronoRoot = "/Users/kylesha/Documents/sbel/chrono-sensor-metal/";
const char* kEsminiRootDefault = "/Users/kylesha/Documents/sbel/esmini";

/// Visual proxy for one scenario actor. esmini drives these kinematically; they carry no
/// collision geometry, so they are moved by SetPos/SetRot rather than simulated.
std::shared_ptr<ChBody> MakeActorProxy(ChSystem& sys, const ChScenarioActor& actor) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName(actor.name.c_str());
    body->SetFixed(true);
    body->EnableCollision(false);

    // The OpenSCENARIO reference point is not the bounding box center, so the box is offset by
    // the center offset the scenario reports rather than centered on the body origin.
    double l = actor.length > 0 ? actor.length : 4.5;
    double w = actor.width > 0 ? actor.width : 1.8;
    double h = actor.height > 0 ? actor.height : 1.5;

    auto box = chrono_types::make_shared<ChVisualShapeBox>(l, w, h);
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(ChColor(0.80f, 0.12f, 0.10f));
    box->SetMaterial(0, mat);

    body->AddVisualShape(box, ChFrame<double>(actor.center_offset, QUNIT));
    sys.Add(body);
    return body;
}
}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    SetChronoDataPath(std::string(kChronoRoot) + "data/");
    vehicle::SetVehicleDataPath(std::string(kChronoRoot) + "data/vehicle/");

    const char* esmini_root_env = std::getenv("ESMINI_ROOT");
    std::string esmini_root = esmini_root_env ? esmini_root_env : kEsminiRootDefault;
    std::string xosc_file = (argc > 1 && argv[1][0] != '\0')
                                ? argv[1]
                                : esmini_root + "/resources/xosc/cut-in_external.xosc";
    double max_time = (argc > 2) ? std::atof(argv[2]) : 30.0;
    double requested_speed = (argc > 3) ? std::atof(argv[3]) : 30.0;

    // ---------------------------------------------------------------------------------------
    // Scenario
    // ---------------------------------------------------------------------------------------

    ChScenarioPlayer player;
    if (!player.Initialize(xosc_file)) {
        printf("Could not load OpenSCENARIO file: %s\n", xosc_file.c_str());
        return 1;
    }

    printf("Loaded %s\n", xosc_file.c_str());
    printf("  %d entities, ego id %d\n", player.GetNumObjects(), player.GetEgoId());
    for (const auto& a : player.GetAllObjects()) {
        printf("    [%d] %-12s %.1f x %.1f x %.1f m%s\n", a.id, a.name.c_str(), a.length, a.width,
               a.height, a.id == player.GetEgoId() ? "   <- ego, driven by Chrono" : "");
    }

    // ---------------------------------------------------------------------------------------
    // Road network referenced by the scenario
    // ---------------------------------------------------------------------------------------

    std::string odr_file = player.GetOdrFilename();
    if (odr_file.empty()) {
        printf("Scenario does not reference an OpenDRIVE file.\n");
        return 1;
    }

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(odr_file)) {
        printf("Could not load the scenario's OpenDRIVE file: %s\n", odr_file.c_str());
        return 1;
    }
    printf("  road network %s (%u roads)\n\n", odr_file.c_str(), network->GetNumRoads());

    // ---------------------------------------------------------------------------------------
    // System, terrain, ego vehicle
    // ---------------------------------------------------------------------------------------

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetMeshResolution(2.0, 2);
    terrain.SetRoadDiffuseTextureFile(GetVehicleDataFile("terrain/textures/tile4.jpg"), 0.2f, 0.2f);
    terrain.CreateVisualizationMesh();

    // Spawn the Audi where and how fast the scenario says the ego starts.
    //
    // The scenario places the ego by its OpenSCENARIO reference point (rear axle center), so the
    // chassis reference frame has to be offset backwards out of it. The offset is purely
    // geometric, so measure it on a throwaway vehicle in a scratch system -- cheap, and it avoids
    // the chicken-and-egg of needing the offset before the real vehicle exists.
    std::string audi_json = GetVehicleDataFile("audi/json/audi_Vehicle.json");
    ChVector3d ref_offset;
    {
        ChSystemNSC probe_sys;
        WheeledVehicle probe(&probe_sys, audi_json);
        probe.Initialize(ChCoordsys<>(VNULL, QUNIT));  // powertrain and tires are not needed
        ref_offset = GetScenarioRefPointOffset(probe);
    }

    ChScenarioActor ego0 = player.GetObject(player.GetEgoId());

    // Place the chassis so that the rear axle center lands on the scenario's reference point.
    ChCoordsys<> spawn = ego0.pose;
    spawn.pos -= spawn.rot.Rotate(ref_offset);
    spawn.pos.z() = ego0.pose.pos.z() + 0.65;  // start above the road; settles to ride height

    // An externally-controlled ego reports back 0 until we tell it otherwise (see the note at the
    // top), so fall back to the requested speed in that case.
    double initial_speed = ego0.speed;
    bool speed_from_scenario = initial_speed > 0.1;
    if (!speed_from_scenario)
        initial_speed = requested_speed;

    printf("Ego starts at road %u lane %d s=%.1f m, %.1f m/s (%s)\n", ego0.road_id, ego0.lane_id,
           ego0.s, initial_speed,
           speed_from_scenario ? "from the scenario" : "supplied here; ego is externally controlled");
    printf("  OpenSCENARIO ref point is %+.3f m from the chassis frame; spawn offset applied\n\n",
           ref_offset.x());

    WheeledVehicle audi(&sys, audi_json);
    audi.Initialize(spawn, initial_speed);
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetWheelVisualizationType(VisualizationType::MESH);
    {
        auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
        auto transmission =
            ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
        audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : audi.GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_Pac02Tire.json"));
                tire->SetStepsize(1e-3);
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    // One-time check that the reference point conversion put the ego where the scenario asked.
    // Reporting the chassis frame instead would place it a wheelbase-dependent distance further
    // along the road, and every trigger distance would inherit that error.
    {
        ChCoordsys<> ref = GetScenarioRefPose(audi);
        double yaw0 = ref.rot.GetCardanAnglesZYX().z();
        ChLaneInfo at_ref = network->GetLaneInfo(ref.pos, yaw0);
        ChLaneInfo at_chassis = network->GetLaneInfo(audi.GetPos(), yaw0);
        printf("  ego s: scenario asked %.2f | ref point %.2f | chassis frame %.2f (error %+.2f m)\n\n",
               ego0.s, at_ref.s, at_chassis.s, at_chassis.s - ego0.s);
    }

    // Hold the ego's starting lane at its starting speed. The scenario does the rest.
    auto path = network->CreateLaneCenterPath(ego0.road_id, ego0.lane_id, 1.0);
    if (!path) {
        printf("Could not extract a center line for the ego's lane.\n");
        return 1;
    }

    ChPathFollowerDriver driver(audi, path, "ego_lane", initial_speed);
    driver.GetSteeringController().SetLookAheadDistance(10.0);
    driver.GetSteeringController().SetGains(0.6, 0.0, 0.0);
    driver.GetSpeedController().SetGains(0.4, 0.01, 0.0);
    driver.Initialize();

    // ---------------------------------------------------------------------------------------
    // Sensors
    // ---------------------------------------------------------------------------------------

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->SetAmbientLight(ChVector3f(0.25f, 0.25f, 0.28f));
    manager->scene->AddPointLight(ChVector3f(20, 10, 25), ChColor(1.0f, 0.98f, 0.92f), 300.f);
    manager->scene->AddPointLight(ChVector3f(-20, -15, 20), ChColor(0.35f, 0.4f, 0.55f), 200.f);

    ChVector3d cam_off(-10.0, 0.0, 3.2);
    ChVector3d look(6.0, 0.0, 0.5);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off,
                             QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 60.0f, cam_pose, 1280, 720,
                                                         (float)(CH_PI / 3), 2);
    cam->SetName("scenario_chase");
    auto vis = chrono_types::make_shared<ChFilterMetalVisualize>(1280, 720, "OpenSCENARIO - chase (Metal RT)");
    cam->PushFilter(vis);
    manager->AddSensor(cam);

    // ---------------------------------------------------------------------------------------
    // Co-simulation loop
    // ---------------------------------------------------------------------------------------

    printf("Running - close the window or wait %.0f s.\n\n", max_time);

    std::map<int, std::shared_ptr<ChBody>> actor_proxies;

    const double step = 1e-3;
    double time = 0;
    double next_report = 0;
    int last_ego_lane = ego0.lane_id;
    auto t_start = std::chrono::steady_clock::now();

    while (vis->WindowOpen() && !player.IsComplete() && time < max_time) {
        double ego_speed = audi.GetSpeed();
        ChCoordsys<> ego_ref = GetScenarioRefPose(audi);

        // Hand Chrono's ego state to the scenario, then let the story advance against it. The
        // overload reports the OpenSCENARIO reference point rather than the chassis frame.
        player.ReportEgoState(audi);
        player.Advance(step);

        // Pull the ambient traffic back into Chrono.
        for (const auto& actor : player.GetActors()) {
            auto it = actor_proxies.find(actor.id);
            if (it == actor_proxies.end())
                it = actor_proxies.emplace(actor.id, MakeActorProxy(sys, actor)).first;

            it->second->SetPos(actor.pose.pos);
            it->second->SetRot(actor.pose.rot);
        }

        DriverInputs in = driver.GetInputs();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);

        driver.Advance(step);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();

        // Report lane-referenced state for the ego and each actor, plus the gap the scenario's
        // cut-in trigger is watching.
        double yaw = ego_ref.rot.GetCardanAnglesZYX().z();
        ChLaneInfo ego_info = network->GetLaneInfo(ego_ref.pos, yaw);

        if (ego_info.valid && ego_info.lane_id != last_ego_lane) {
            printf("t=%5.2f s  *** ego changed lane %+d -> %+d ***\n", time, last_ego_lane,
                   ego_info.lane_id);
            last_ego_lane = ego_info.lane_id;
        }

        if (time >= next_report) {
            printf("t=%5.1f s  ego: lane %+d s=%7.1f v=%5.1f m/s", time, ego_info.lane_id, ego_info.s,
                   ego_speed);
            for (const auto& actor : player.GetActors()) {
                // Measure the gap the way the scenario's TimeHeadwayCondition does: along the
                // road between reference points, signed positive when the actor is ahead. A 3D
                // straight-line distance would fold in lateral and vertical separation, which
                // that condition does not consider.
                bool same_road = ego_info.valid && actor.road_id == ego_info.road_id;
                double gap = same_road ? actor.s - ego_info.s
                                       : (actor.pose.pos - ego_ref.pos).Length();
                double headway = ego_speed > 0.1 ? gap / ego_speed : 0.0;
                printf(" | %s: lane %+d s=%7.1f v=%5.1f gap=%+6.1f m thw=%+5.2f s%s",
                       actor.name.c_str(), actor.lane_id, actor.s, actor.speed, gap, headway,
                       same_road ? "" : " (3D)");
            }
            printf("\n");
            next_report += 0.5;
        }

        time += step;
        double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        if (time > wall)
            std::this_thread::sleep_for(std::chrono::duration<double>(time - wall));
    }

    printf("\nStopped at t = %.2f s%s\n", time, player.IsComplete() ? " (scenario complete)" : "");
    return 0;
}
