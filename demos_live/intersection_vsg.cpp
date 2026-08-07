// An OpenSCENARIO scenario at a real urban intersection, drawn with Chrono::VSG.
//
// Road network: fabriksgatan.xodr, a real junction in Sweden -- 16 roads, 12 of them junction
// connectors. The ego drives straight through the junction while the scenario puts something in
// its way.
//
// The ego's path comes from ChOpenDriveNetwork::CreateRoutePath, which walks the lane graph
// across the junction. A per-road center line would simply end at the junction mouth, so this is
// what makes driving through an intersection expressible at all. The yellow line in the window is
// that path: Chrono's closed-loop driver draws its own reference, so it doubles as a check that
// the lane graph came out right.
//
// On the ego speed argument: ltap-od.xosc assigns an interactiveDriver controller to its ego, so
// it is authored for a human at the keyboard, and the NPC's SynchronizeAction times its turn
// against the ego's arrival. How fast the ego approaches therefore decides whether the two meet
// in the junction at all.
//
// Usage:  ./intersection_vsg [scenario.xosc] [max_seconds] [ego_speed_mps]
//
// Defaults to ltap-od.xosc. Any fabriksgatan scenario whose ego follows
// HostStraightRoute works unchanged -- pedestrian_collision.xosc and pedestrian.xosc
// both do, and put a crossing pedestrian in the junction instead of a turning car.
// Env:    ESMINI_ROOT  esmini resource root
//         KEEP_OPEN=1  ignore the scenario's stop trigger and max_seconds, and keep simulating
//                      until the window is closed
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_scenario/ChOpenDriveNetwork.h"
#include "chrono_scenario/ChOpenDriveTerrain.h"
#include "chrono_scenario/ChScenarioActorShapes.h"
#include "chrono_scenario/ChScenarioPlayer.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::scenario;

namespace {
const char* kChronoRoot = "/Users/kylesha/Documents/sbel/chrono-sensor-metal/";
const char* kEsminiRootDefault = "/Users/kylesha/Documents/sbel/esmini";

// From RoutesAtFabriksgatan.xosc: the ego's HostStraightRoute runs road 0 lane +1 into road 2.
constexpr unsigned int kEgoRoad = 0;
constexpr int kEgoLane = 1;
constexpr double kEgoStartS = 63.0;
// Long enough that the ego does not run off the end of its path while the window is
// still open; road 2 continues well past the junction.
constexpr double kRouteLength = 280.0;

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    SetChronoDataPath(std::string(kChronoRoot) + "data/");
    vehicle::SetVehicleDataPath(std::string(kChronoRoot) + "data/vehicle/");

    const char* esmini_root_env = std::getenv("ESMINI_ROOT");
    std::string esmini_root = esmini_root_env ? esmini_root_env : kEsminiRootDefault;
    std::string xosc_file = (argc > 1 && argv[1][0] != '\0')
                                ? argv[1]
                                : esmini_root + "/resources/xosc/ltap-od.xosc";
    double max_time = (argc > 2) ? std::atof(argv[2]) : 25.0;
    double ego_speed_target = (argc > 3) ? std::atof(argv[3]) : 10.0;  // HostSpeed in ltap-od.xosc
    bool keep_open = std::getenv("KEEP_OPEN") != nullptr;

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

    printf("Loaded %s\n  network: %u roads, ego target speed %.1f m/s\n", xosc_file.c_str(),
           network->GetNumRoads(), ego_speed_target);
    for (const auto& a : player.GetAllObjects())
        printf("    [%d] %-18s %.1f x %.1f x %.1f m  type=%d cat=%d%s\n", a.id, a.name.c_str(),
               a.length, a.width, a.height, a.object_type, a.object_category,
               a.id == player.GetEgoId() ? "   <- ego, driven by Chrono" : "");

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
    terrain.SetMeshResolution(1.0, 1);
    terrain.SetRoadDiffuseTextureFile(GetVehicleDataFile("terrain/textures/concrete.jpg"), 0.35f, 0.35f);
    terrain.CreateVisualizationMesh();
    terrain.CreateLaneMarkings();
    printf("  road mesh %zu tris | markings %zu tris\n",
           terrain.GetMesh() ? terrain.GetMesh()->GetIndicesVertices().size() : 0,
           terrain.GetLaneMarkingMesh() ? terrain.GetLaneMarkingMesh()->GetIndicesVertices().size() : 0);

    // ---------------------------------------------------------------------------------------
    // Ego on a route that crosses the junction
    // ---------------------------------------------------------------------------------------

    ChLaneCoord ego_start{kEgoRoad, kEgoLane, kEgoStartS, 0.0};
    auto path = network->CreateRoutePath(ego_start, kRouteLength, ChJunctionChoice::STRAIGHT, 1.0);
    if (!path) {
        printf("Could not build a route through the junction.\n");
        return 1;
    }

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
    // VSG run-time visualization
    // ---------------------------------------------------------------------------------------

    // Visual proxies for the scenario's actors.
    //
    // These must exist *before* any visualization system is created. VSG binds the bodies it
    // finds when its scene graph is built and a later addition is simply never drawn, and
    // Chrono::Sensor likewise will not pick up a mesh attached after its scene is built. The
    // scenario's entity list is known as soon as it loads, so there is no reason to defer.
    std::map<int, std::shared_ptr<ChBody>> actor_proxies;
    for (const auto& actor : player.GetActors())
        actor_proxies[actor.id] = CreateScenarioActorBody(sys, actor);

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->AttachVehicle(&audi);
    vis->AttachDriver(&driver);
    vis->SetWindowTitle("OpenSCENARIO - left turn across path (VSG)");
    vis->SetWindowSize(1400, 850);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.0), 13.0, 1.0);
    vis->Initialize();

    printf("\nRunning%s.\n\n", keep_open ? " until the window is closed" : "");

    // ---------------------------------------------------------------------------------------
    // Co-simulation loop
    // ---------------------------------------------------------------------------------------

    const double step = 1e-3;
    // Physics needs 1 kHz; the display does not. Rendering every physics step draws a thousand
    // frames per simulated second, which costs far more than the geometry does.
    const double render_step = 1.0 / 50;
    double next_render = 0;
    double next_report = 0;
    bool announced_junction = false;
    double closest_approach = 1e9;

    audi.EnableRealtime(true);

    while (vis->Run()) {
        double time = sys.GetChTime();
        if (!keep_open && (time >= max_time || player.IsComplete()))
            break;

        if (time >= next_render) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            next_render += render_step;
        }

        ChCoordsys<> ego_ref = GetScenarioRefPose(audi);
        player.ReportEgoState(audi);
        player.Advance(step);

        for (const auto& actor : player.GetActors()) {
            auto it = actor_proxies.find(actor.id);
            if (it != actor_proxies.end()) {
                it->second->SetPos(actor.pose.pos);
                it->second->SetRot(actor.pose.rot);
            }
            closest_approach = std::min(closest_approach, (actor.pose.pos - ego_ref.pos).Length());
        }

        DriverInputs in = driver.GetInputs();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        vis->Synchronize(time, in);

        driver.Advance(step);
        terrain.Advance(step);
        audi.Advance(step);
        vis->Advance(step);
        sys.DoStepDynamics(step);

        double yaw = ego_ref.rot.GetCardanAnglesZYX().z();
        ChLaneInfo info = network->GetLaneInfo(ego_ref.pos, yaw);

        if (info.valid && info.InJunction() && !announced_junction) {
            printf("t=%5.2f s  *** ego entered the junction ***\n", time);
            announced_junction = true;
        }

        if (time >= next_report) {
            printf("t=%5.1f s  ego: road %2u lane %+d s=%6.1f v=%5.1f m/s%s", time, info.road_id,
                   info.lane_id, info.s, audi.GetSpeed(), info.InJunction() ? "  [in junction]" : "");
            for (const auto& actor : player.GetActors())
                printf(" | %s: road %2u lane %+d s=%6.1f v=%5.1f dist=%5.1f m", actor.name.c_str(),
                       actor.road_id, actor.lane_id, actor.s, actor.speed,
                       (actor.pose.pos - ego_ref.pos).Length());
            printf("\n");
            next_report += 0.5;
        }
    }

    printf("\nStopped at t = %.2f s | closest approach to the NPC: %.1f m\n", sys.GetChTime(),
           closest_approach);
    return 0;
}
