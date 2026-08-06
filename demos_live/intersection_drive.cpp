// Left Turn Across Path at a real urban intersection, rendered.
//
// The road network is fabriksgatan.xodr -- a real junction in Sweden, 16 roads of which 12 are
// junction connectors. The scenario is esmini's ltap-od.xosc: the Chrono-driven ego runs straight
// through the junction while an NPC turns left across its path.
//
// The ego's path comes from ChOpenDriveNetwork::CreateRoutePath, which walks the lane graph
// forward across the junction rather than stopping at the end of a road. That is what makes
// driving through an intersection expressible at all -- a per-road center line would end at the
// junction mouth.
//
// Two cameras: a chase camera on the ego, and a fixed elevated camera looking down at the
// junction. Both write frames, so a run leaves a visual record.
//
// Usage:  ./intersection_drive [max_seconds]
// Env:    ESMINI_ROOT       esmini resource root
//         HEADLESS=1        skip the interactive window, just write frames
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
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/metal/ChFilterMetalVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::scenario;
using namespace chrono::sensor;

namespace {
const char* kChronoRoot = "/Users/kylesha/Documents/sbel/chrono-sensor-metal/";
const char* kEsminiRootDefault = "/Users/kylesha/Documents/sbel/esmini";

// From RoutesAtFabriksgatan.xosc: the ego's HostStraightRoute starts on road 0, lane +1, and
// continues into road 2 through the junction.
constexpr unsigned int kEgoRoad = 0;
constexpr int kEgoLane = 1;
constexpr double kEgoStartS = 63.0;
constexpr double kRouteLength = 130.0;

std::shared_ptr<ChBody> MakeActorProxy(ChSystem& sys, const ChScenarioActor& actor) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName(actor.name.c_str());
    body->SetFixed(true);
    body->EnableCollision(false);

    double l = actor.length > 0 ? actor.length : 4.5;
    double w = actor.width > 0 ? actor.width : 1.8;
    double h = actor.height > 0 ? actor.height : 1.5;

    auto box = chrono_types::make_shared<ChVisualShapeBox>(l, w, h);
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(ChColor(0.85f, 0.10f, 0.08f));
    box->SetMaterial(0, mat);

    // The OpenSCENARIO reference point is the rear axle center, not the box center.
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
    double max_time = (argc > 1) ? std::atof(argv[1]) : 20.0;
    bool headless = std::getenv("HEADLESS") != nullptr;

    std::string xosc_file = esmini_root + "/resources/xosc/ltap-od.xosc";
    std::string out_dir = std::string(kChronoRoot) + "demos_live/intersection_out/";

    // ---------------------------------------------------------------------------------------
    // Scenario and its road network
    // ---------------------------------------------------------------------------------------

    ChScenarioPlayer player;
    if (!player.Initialize(xosc_file)) {
        printf("Could not load %s\n", xosc_file.c_str());
        return 1;
    }

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(player.GetOdrFilename())) {
        printf("Could not load the scenario's road network.\n");
        return 1;
    }

    printf("Loaded %s\n", xosc_file.c_str());
    printf("  network: %u roads\n", network->GetNumRoads());
    for (const auto& a : player.GetAllObjects())
        printf("    [%d] %-8s%s\n", a.id, a.name.c_str(),
               a.id == player.GetEgoId() ? "  <- ego, driven by Chrono" : "  <- turns left across the ego");

    // ---------------------------------------------------------------------------------------
    // System and terrain
    // ---------------------------------------------------------------------------------------

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetMeshResolution(1.0, 3);
    // UVs are emitted in meters, so the scale here is tiles-per-meter.
    terrain.SetRoadDiffuseTextureFile(GetVehicleDataFile("terrain/textures/concrete.jpg"), 0.35f, 0.35f);
    terrain.CreateVisualizationMesh();
    terrain.CreateLaneMarkings();
    printf("  road mesh: %zu vertices", terrain.GetMesh()->GetCoordsVertices().size());
    if (auto mm = terrain.GetLaneMarkingMesh())
        printf(" | lane markings: %zu vertices from %zu marked borders",
               mm->GetCoordsVertices().size(), network->GetMarkedLanes().size());
    printf("\n");

    // ---------------------------------------------------------------------------------------
    // Ego on a route that crosses the junction
    // ---------------------------------------------------------------------------------------

    ChLaneCoord ego_start{kEgoRoad, kEgoLane, kEgoStartS, 0.0};
    auto path = network->CreateRoutePath(ego_start, kRouteLength, ChJunctionChoice::STRAIGHT, 1.0);
    if (!path) {
        printf("Could not build a route through the junction.\n");
        return 1;
    }
    printf("  ego route: %zu points over %.0f m, straight through the junction\n",
           network->SampleRoute(ego_start, kRouteLength, ChJunctionChoice::STRAIGHT, 1.0).size(),
           kRouteLength);

    std::string audi_json = GetVehicleDataFile("audi/json/audi_Vehicle.json");
    ChVector3d ref_offset;
    {
        ChSystemNSC probe_sys;
        WheeledVehicle probe(&probe_sys, audi_json);
        probe.Initialize(ChCoordsys<>(VNULL, QUNIT));
        ref_offset = GetScenarioRefPointOffset(probe);
    }

    ChCoordsys<> spawn = network->LaneToWorld(ego_start, false);
    spawn.pos -= spawn.rot.Rotate(ref_offset);
    spawn.pos.z() += 0.65;

    const double ego_speed_target = 10.0;  // HostSpeed in ltap-od.xosc

    WheeledVehicle audi(&sys, audi_json);
    audi.Initialize(spawn, ego_speed_target);
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

    ChPathFollowerDriver driver(audi, path, "ego_route", ego_speed_target);
    driver.GetSteeringController().SetLookAheadDistance(6.0);
    driver.GetSteeringController().SetGains(0.8, 0.0, 0.0);
    driver.GetSpeedController().SetGains(0.4, 0.01, 0.0);
    driver.Initialize();

    // ---------------------------------------------------------------------------------------
    // Cameras
    //
    // The junction center is taken from the route: the ego passes through it, so a point partway
    // along the sampled route is a reliable place to aim the fixed camera without hardcoding
    // world coordinates for this particular map.
    // ---------------------------------------------------------------------------------------

    auto route_pts = network->SampleRoute(ego_start, kRouteLength, ChJunctionChoice::STRAIGHT, 1.0);
    ChVector3d junction = route_pts[route_pts.size() / 2];

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    // Without an environment light the road sits against pure black, since OpenDRIVE supplies no
    // surroundings of any kind -- no sky, no ground plane, no buildings.
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.38f));
    manager->scene->AddPointLight(ChVector3f((float)junction.x() + 40, (float)junction.y() + 30, 45),
                                  ChColor(1.0f, 0.98f, 0.92f), 600.f);
    manager->scene->AddPointLight(ChVector3f((float)junction.x() - 40, (float)junction.y() - 30, 40),
                                  ChColor(0.45f, 0.5f, 0.62f), 400.f);

    // Chase camera on the ego.
    ChVector3d cam_off(-9.0, 0.0, 3.0);
    ChVector3d look(5.0, 0.0, 0.5);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> chase_pose(cam_off,
                               QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto chase = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 10.0f, chase_pose, 1280,
                                                           720, (float)(CH_PI / 3), 2);
    chase->SetName("intersection_chase");
    chase->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "chase/"));
    std::shared_ptr<ChFilterMetalVisualize> chase_vis;
    if (!headless) {
        chase_vis = chrono_types::make_shared<ChFilterMetalVisualize>(1280, 720, "Intersection - chase");
        chase->PushFilter(chase_vis);
    }
    manager->AddSensor(chase);

    // Fixed camera above the junction, looking down at it. Attached to the (static) ground body
    // so its pose is world-fixed.
    ChVector3d eye = junction + ChVector3d(-28.0, -28.0, 26.0);
    ChVector3d dir = (junction - eye).GetNormalized();
    ChFrame<double> over_pose(eye, QuatFromAngleZ(std::atan2(dir.y(), dir.x())) *
                                       QuatFromAngleY(-std::asin(dir.z())));
    auto overhead = chrono_types::make_shared<ChCameraSensor>(terrain.GetGround(), 10.0f, over_pose, 1280,
                                                              720, (float)(CH_PI / 2.6), 2);
    overhead->SetName("intersection_overhead");
    overhead->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "overhead/"));
    std::shared_ptr<ChFilterMetalVisualize> over_vis;
    if (!headless) {
        over_vis = chrono_types::make_shared<ChFilterMetalVisualize>(1280, 720, "Intersection - overhead");
        overhead->PushFilter(over_vis);
    }
    manager->AddSensor(overhead);

    // ---------------------------------------------------------------------------------------
    // Co-simulation loop
    // ---------------------------------------------------------------------------------------

    printf("\nRunning %.0f s%s. Frames -> %s\n\n", max_time, headless ? " (headless)" : "",
           out_dir.c_str());

    std::map<int, std::shared_ptr<ChBody>> actor_proxies;
    const double step = 1e-3;
    double time = 0;
    double next_report = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (time < max_time && !player.IsComplete()) {
        if (!headless && chase_vis && !chase_vis->WindowOpen())
            break;

        ChCoordsys<> ego_ref = GetScenarioRefPose(audi);
        player.ReportEgoState(audi);
        player.Advance(step);

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

        if (time >= next_report) {
            double yaw = ego_ref.rot.GetCardanAnglesZYX().z();
            ChLaneInfo info = network->GetLaneInfo(ego_ref.pos, yaw);
            printf("t=%5.1f s  ego: road %2u lane %+d s=%6.1f v=%5.1f m/s%s", time, info.road_id,
                   info.lane_id, info.s, audi.GetSpeed(), info.InJunction() ? "  [in junction]" : "");
            for (const auto& actor : player.GetActors())
                printf(" | %s: road %2u lane %+d s=%6.1f v=%5.1f dist=%5.1f m", actor.name.c_str(),
                       actor.road_id, actor.lane_id, actor.s, actor.speed,
                       (actor.pose.pos - ego_ref.pos).Length());
            printf("\n");
            next_report += 0.5;
        }

        time += step;
        if (!headless) {
            double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
            if (time > wall)
                std::this_thread::sleep_for(std::chrono::duration<double>(time - wall));
        }
    }

    printf("\nStopped at t = %.2f s. Frames written to %s\n", time, out_dir.c_str());
    return 0;
}
