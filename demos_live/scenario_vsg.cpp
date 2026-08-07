// An ASAM OpenSCENARIO scenario run in co-simulation with Chrono, drawn with Chrono::VSG.
//
// Same coupling as scenario_drive.cpp -- esmini owns the story and the ambient traffic, Chrono
// owns the ego's dynamics -- but rendered by the rasterizing VSG visualizer instead of the
// ray-traced sensor pipeline.
//
// That distinction matters for watching a run. Chrono::Sensor exists to simulate cameras, and it
// pays ray-tracing cost per pixel for physical fidelity nobody needs when the point is simply to
// see what happened. On e6mini the road and its markings come to roughly 72k triangles, which
// dropped the sensor-rendered demo to 0.27x real time; the same scene rasterizes essentially for
// free. Use scenario_drive.cpp when a simulated camera is the point, and this when it is not.
//
// Usage:  ./scenario_vsg [path/to/scenario.xosc] [max_seconds] [ego_speed_mps]
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
    bool speed_given = argc > 3;
    double requested_speed = speed_given ? std::atof(argv[3]) : 30.0;
    bool keep_open = std::getenv("KEEP_OPEN") != nullptr;

    // ---------------------------------------------------------------------------------------
    // Scenario and its road network
    // ---------------------------------------------------------------------------------------

    ChScenarioPlayer player;
    if (!player.Initialize(xosc_file)) {
        printf("Could not load OpenSCENARIO file: %s\n", xosc_file.c_str());
        return 1;
    }

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(player.GetOdrFilename())) {
        printf("Could not load the scenario's OpenDRIVE file.\n");
        return 1;
    }

    printf("Loaded %s\n", xosc_file.c_str());
    printf("  %d entities, ego id %d | network %u roads\n", player.GetNumObjects(), player.GetEgoId(),
           network->GetNumRoads());
    for (const auto& a : player.GetAllObjects()) {
        const char* kind = a.object_type == ChScenarioObjectType::VEHICLE      ? "vehicle"
                           : a.object_type == ChScenarioObjectType::PEDESTRIAN ? "pedestrian"
                           : a.object_type == ChScenarioObjectType::MISC_OBJECT ? "misc"
                                                                                : "unknown";
        printf("    [%d] %-18s %-10s cat=%-2d %.1f x %.1f x %.1f m%s\n", a.id, a.name.c_str(), kind,
               a.object_category, a.length, a.width, a.height,
               a.id == player.GetEgoId() ? "   <- ego, driven by Chrono" : "");
    }

    // ---------------------------------------------------------------------------------------
    // System, terrain, ego
    // ---------------------------------------------------------------------------------------

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetMeshResolution(2.0, 1);
    terrain.SetRoadDiffuseTextureFile(GetVehicleDataFile("terrain/textures/concrete.jpg"), 0.35f, 0.35f);
    terrain.CreateVisualizationMesh();
    terrain.CreateLaneMarkings();
    printf("  road mesh %zu tris | markings %zu tris\n",
           terrain.GetMesh() ? terrain.GetMesh()->GetIndicesVertices().size() : 0,
           terrain.GetLaneMarkingMesh() ? terrain.GetLaneMarkingMesh()->GetIndicesVertices().size() : 0);

    std::string audi_json = GetVehicleDataFile("audi/json/audi_Vehicle.json");
    ChVector3d ref_offset;
    {
        ChSystemNSC probe_sys;
        WheeledVehicle probe(&probe_sys, audi_json);
        probe.Initialize(ChCoordsys<>(VNULL, QUNIT));
        ref_offset = GetScenarioRefPointOffset(probe);
    }

    ChScenarioActor ego0 = player.GetObject(player.GetEgoId());
    ChCoordsys<> spawn = ego0.pose;
    spawn.pos -= spawn.rot.Rotate(ref_offset);
    spawn.pos.z() = ego0.pose.pos.z() + 0.65;

    // An explicit argument always wins. Otherwise take the scenario's own initial speed, and
    // fall back to the default only when it reports none -- which is what an externally
    // controlled ego does, since esmini leaves its Init SpeedAction unapplied.
    double initial_speed = speed_given ? requested_speed
                                       : (ego0.speed > 0.1 ? ego0.speed : requested_speed);
    printf("  ego starts road %u lane %+d s=%.1f, target %.1f m/s (%s)\n", ego0.road_id, ego0.lane_id,
           ego0.s, initial_speed,
           speed_given ? "from the command line"
                       : (ego0.speed > 0.1 ? "from the scenario" : "default; scenario reports none"));

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
    vis->SetWindowTitle("OpenSCENARIO co-simulation (VSG)");
    vis->SetWindowSize(1400, 850);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.0), 11.0, 0.6);
    vis->Initialize();

    printf("\nRunning%s. Drag to orbit; the chase camera follows the ego.\n\n",
           keep_open ? " until the window is closed" : "");

    // ---------------------------------------------------------------------------------------
    // Co-simulation loop
    // ---------------------------------------------------------------------------------------

    const double step = 1e-3;
    // Physics runs at 1 kHz because the tires need it; the display does not. Rendering every
    // physics step would draw a thousand frames per simulated second and is what makes these
    // loops miss real time -- the cost is in the frame count, not the triangle count.
    const double render_step = 1.0 / 50;
    double next_render = 0;
    int frame_idx = 0;
    double next_report = 0;

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
            // Opt-in frame capture, so a run can be checked without watching it.
            if (std::getenv("SAVE_FRAMES")) {
                char fn[512];
                snprintf(fn, sizeof(fn), "%sdemos_live/vsg_out/frame_%04d.png", kChronoRoot, frame_idx);
                vis->WriteImageToFile(fn);
            }
            frame_idx++;
        }

        ChCoordsys<> ego_ref = GetScenarioRefPose(audi);
        double ego_speed = audi.GetSpeed();

        player.ReportEgoState(audi);
        player.Advance(step);

        for (const auto& actor : player.GetActors()) {
            auto it = actor_proxies.find(actor.id);
            if (it != actor_proxies.end()) {
                it->second->SetPos(actor.pose.pos);
                it->second->SetRot(actor.pose.rot);
            }
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

        if (time >= next_report) {
            double yaw = ego_ref.rot.GetCardanAnglesZYX().z();
            ChLaneInfo info = network->GetLaneInfo(ego_ref.pos, yaw);
            printf("t=%5.1f s  ego: lane %+d s=%7.1f v=%5.1f m/s xy=(%7.1f,%6.1f)", time, info.lane_id,
                   info.s, ego_speed, ego_ref.pos.x(), ego_ref.pos.y());
            for (const auto& actor : player.GetActors()) {
                bool same_road = info.valid && actor.road_id == info.road_id;
                double gap = same_road ? actor.s - info.s : (actor.pose.pos - ego_ref.pos).Length();
                double a_yaw = actor.pose.rot.GetCardanAnglesZYX().z() * 180.0 / CH_PI;
                printf(" | %s: lane %+d gap=%+6.1f m xy=(%7.1f,%6.1f) yaw=%+6.1f deg",
                       actor.name.c_str(), actor.lane_id, gap, actor.pose.pos.x(),
                       actor.pose.pos.y(), a_yaw);
            }
            printf("\n");
            next_report += 0.5;
        }
    }

    printf("\nStopped at t = %.2f s\n", sys.GetChTime());
    return 0;
}
