// Driving the Mcity digital twin in Chrono.
//
// Mcity is the University of Michigan's AV proving ground, and its digital twin is published
// under an MIT licence at github.com/mcity/mcity-digital-twin. Two things come out of that repo,
// and this demo uses each for what it is actually good at:
//
//   McityMap_Main.xodr   the ASAM OpenDRIVE network: 411 roads, 45 junctions, 8.0 km of road,
//                        1172 road marks, real elevation. Used for the *driving surface* and for
//                        *semantics* -- which lane am I in, where does this lane go, how do I get
//                        through this junction. The surface is answered analytically, so tire
//                        contact never sees a tessellation.
//
//   Omniverse/*.usdc     the 3D environment as a composition: 792 placements drawing on 139
//                        distinct meshes -- roads, lane markings, ground, buildings, poles,
//                        signal heads, barriers, street lights. Used for *everything visible*.
//
// The two register without a fitted transform. Checked by matching each OpenDRIVE signal against
// the nearest USD prop: the signed offsets scatter about zero (dx mean -0.92 m against 2.56 m
// spread, dy +1.06 against 3.28) rather than showing the constant bias a frame mismatch produces.
// The residual is signal-head-versus-pole geometry, not registration error.
//
// Note what is deliberately *not* drawn: the road surface Chrono generates from OpenDRIVE. Mcity
// ships its own authored road and lane-marking meshes, which look better and are what the
// facility actually looks like. Drawing both would z-fight. So OpenDRIVE is invisible here and
// purely functional -- it is what the tires stand on and what the driver steers by.
//
// Setup:  ./demos_live/mcity/fetch_mcity.sh && ./demos_live/mcity/usd_to_chrono.py
//
// Usage:  ./mcity_drive_vsg [seconds] [speed_mps]
// Env:    MCITY_DIR    asset directory (default data/mcity)
//         KEEP_OPEN=1  run until the window is closed
//         NO_SCENERY=1 road network only, for comparison
//         LIGHT=1      drop the heaviest props (perimeter fence, GPS blocker)
//         SAVE_FRAMES  write frames to demos_live/mcity_out/ (costs roughly 13x real time)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

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
#include "chrono_scenario/ChSceneryModel.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::scenario;

namespace {
const char* kChronoRoot = "/Users/kylesha/Documents/sbel/chrono-sensor-metal/";

/// Pick a starting lane worth driving: the longest road that carries a drivable lane.
///
/// Mcity averages ~20 m per road across 411 of them, most being junction connectors, so the
/// first road in the file is a poor default. Taking the longest gives a straight worth driving
/// before the route reaches a junction.
bool ChooseStart(const std::shared_ptr<ChOpenDriveNetwork>& net, ChLaneCoord& start, double& road_len) {
    double best = 0;
    bool found = false;
    for (unsigned int road : net->GetRoadIds()) {
        double len = net->GetRoadLength(road);
        if (len <= best)
            continue;
        auto lanes = net->GetLaneIds(road, 0.5 * len);
        if (lanes.empty())
            continue;

        // Prefer a right-hand lane; Mcity is a US facility and drives on the right.
        int lane = lanes.front();
        for (int id : lanes) {
            if (id < 0) {
                lane = id;
                break;
            }
        }
        best = len;
        road_len = len;
        start = ChLaneCoord{road, lane, 0.1 * len, 0.0};
        found = true;
    }
    return found;
}
}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    SetChronoDataPath(std::string(kChronoRoot) + "data/");
    vehicle::SetVehicleDataPath(std::string(kChronoRoot) + "data/vehicle/");

    const char* dir_env = std::getenv("MCITY_DIR");
    std::string mcity_dir = dir_env ? dir_env : std::string(kChronoRoot) + "data/mcity";
    double max_time = (argc > 1) ? std::atof(argv[1]) : 60.0;
    double target_speed = (argc > 2) ? std::atof(argv[2]) : 8.0;
    bool keep_open = std::getenv("KEEP_OPEN") != nullptr;
    bool no_scenery = std::getenv("NO_SCENERY") != nullptr;
    bool light = std::getenv("LIGHT") != nullptr;

    // ---------------------------------------------------------------------------------------
    // Road network: contact surface and lane semantics
    // ---------------------------------------------------------------------------------------

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(mcity_dir + "/McityMap_Main.xodr")) {
        printf("Could not load the Mcity road network from %s\n", mcity_dir.c_str());
        printf("Run demos_live/mcity/fetch_mcity.sh first.\n");
        return 1;
    }

    double total_length = 0;
    for (unsigned int r : network->GetRoadIds())
        total_length += network->GetRoadLength(r);
    printf("Mcity\n  network: %u roads, %.1f km, %zu marked lane borders\n", network->GetNumRoads(),
           total_length / 1000.0, network->GetMarkedLanes().size());

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    // The terrain answers height, normal and friction from the road geometry. Its visualization
    // mesh is deliberately never built -- Mcity's own road mesh is drawn instead.
    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);

    // ---------------------------------------------------------------------------------------
    // Scenery: everything visible
    // ---------------------------------------------------------------------------------------

    ChSceneryModel scenery;
    if (!no_scenery) {
        ChSceneryModel::Options opts;
        if (light) {
            // The perimeter fence alone is 169k triangles and the GPS blocker another 52k, both
            // placed once and neither of interest from inside a car.
            opts.max_triangles_per_asset = 60000;
        }
        if (!scenery.Load(sys, mcity_dir + "/mcity_scene.json", opts)) {
            printf("  no scenery manifest; run demos_live/mcity/usd_to_chrono.py\n");
        } else {
            scenery.ReportTo(std::cout);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Ego on a route through the network
    // ---------------------------------------------------------------------------------------

    ChLaneCoord start{0, -1, 0.0, 0.0};
    double road_len = 0;
    if (!ChooseStart(network, start, road_len)) {
        printf("No drivable lane found in the network.\n");
        return 1;
    }

    // Route across junctions rather than stopping at the end of a road, since Mcity's roads
    // average about 20 m and a per-road path would end almost immediately.
    auto path = network->CreateRoutePath(start, 600.0, ChJunctionChoice::STRAIGHT, 1.0);
    if (!path) {
        printf("Could not build a route from road %u lane %d.\n", start.road_id, start.lane_id);
        return 1;
    }
    auto route_pts = network->SampleRoute(start, 600.0, ChJunctionChoice::STRAIGHT, 1.0);
    printf("  route: %zu points from road %u lane %+d (longest road, %.0f m)\n", route_pts.size(),
           start.road_id, start.lane_id, road_len);

    std::string audi_json = GetVehicleDataFile("audi/json/audi_Vehicle.json");
    ChCoordsys<> spawn = network->LaneToWorld(start, false);
    spawn.pos.z() += 0.30;  // settle onto the road rather than dropping from height

    WheeledVehicle audi(&sys, audi_json);
    audi.Initialize(spawn, target_speed);
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

    ChPathFollowerDriver driver(audi, path, "mcity_route", target_speed, 0.5, 2.0);
    driver.GetSteeringController().SetLookAheadDistance(6.0);
    driver.GetSteeringController().SetGains(0.8, 0.0, 0.0);
    driver.GetSpeedController().SetGains(0.4, 0.01, 0.0);
    driver.Initialize();

    // ---------------------------------------------------------------------------------------
    // Visualization
    // ---------------------------------------------------------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->AttachVehicle(&audi);
    vis->AttachDriver(&driver);
    vis->SetWindowTitle("Mcity digital twin - Chrono");
    vis->SetWindowSize(1500, 900);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.2), 12.0, 0.7);
    vis->Initialize();

    printf("\nDriving Mcity at %.1f m/s%s.\n\n", target_speed,
           keep_open ? ", until the window is closed" : "");

    // ---------------------------------------------------------------------------------------
    // Simulation loop
    // ---------------------------------------------------------------------------------------

    const double step = 1e-3;
    const double render_step = 1.0 / 50;  // physics needs 1 kHz; the display does not
    double next_render = 0;
    double next_report = 0;
    bool save_frames = std::getenv("SAVE_FRAMES") != nullptr;
    int frame_idx = 0;

    audi.EnableRealtime(true);

    while (vis->Run()) {
        double time = sys.GetChTime();
        if (!keep_open && time >= max_time)
            break;

        if (time >= next_render) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            next_render += render_step;
            if (save_frames) {
                char fn[512];
                snprintf(fn, sizeof(fn), "%sdemos_live/mcity_out/frame_%04d.png", kChronoRoot, frame_idx);
                vis->WriteImageToFile(fn);
            }
            frame_idx++;
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
            double yaw = audi.GetRot().GetCardanAnglesZYX().z();
            ChLaneInfo info = network->GetLaneInfo(audi.GetPos(), yaw);
            printf("t=%5.1f s  v=%5.1f m/s  z=%6.1f m", time, audi.GetSpeed(), audi.GetPos().z());
            if (info.valid)
                printf("  road %3u lane %+d s=%6.1f offset=%+5.2f%s", info.road_id, info.lane_id,
                       info.s, info.lane_offset, info.InJunction() ? "  [junction]" : "");
            else
                printf("  off the network");
            printf("\n");
            next_report += 1.0;
        }
    }

    printf("\nStopped at t = %.2f s\n", sys.GetChTime());
    return 0;
}
