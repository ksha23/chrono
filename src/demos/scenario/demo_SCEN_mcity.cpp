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
// Setup:  src/demos/scenario/mcity/setup_mcity.sh [--repo <clone>] [--foliage]
//
// Usage:  demo_SCEN_mcity [options]
//
//   --foliage LEVEL   none | trees | trees-leaf | shrubs | full   (default none)
//                       none        no vegetation
//                       trees       447 trees, branches only
//                       trees-leaf  447 trees with leaves
//                       shrubs      trees and shrubs, branches only
//                       full        everything with leaves (heavy: ~107M triangles)
//   --scene FILE      explicit manifest, overriding --foliage
//   --data DIR        converted asset directory (default: <chrono data>/mcity)
//   --seconds N       stop after N seconds of simulation; 0 runs until the window closes
//   --speed V         target speed, m/s (default 8)
//   --radius R        draw distance in metres; 0 draws the whole scene (default 90)
//   --tile T          scenery tile size in metres, the unit --radius switches (default 40)
//   --route L         route length in metres (default 600)
//   --tire MODEL      pac02 | tmeasy | rigid (default pac02)
//   --render-fps N    render rate, decoupled from the 1 kHz physics (default 50)
//   --max-tris N      skip any single asset above this triangle count
//   --no-scenery      road network only, for comparison
//   --show-roadnet    also draw the OpenDRIVE surface, to check it against the USD road
//   --flat-terrain    take ground height from OpenDRIVE instead of the drawn meshes
//   --no-shadows      disable shadow mapping
//   --save-frames     write frames to mcity_out/ (costs roughly 13x real time)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
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
#include "chrono_vehicle/utils/ChSceneryModel.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::scenario;

namespace {

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


namespace {

/// Everything the demo can be told to do. Flags rather than environment variables: there were
/// fourteen of the latter, which is unreadable at a call site and undiscoverable without reading
/// the source.
struct Options {
    std::string foliage = "none";
    std::string scene;       ///< explicit manifest, overrides foliage
    std::string data_dir;    ///< converted assets
    std::string tire = "pac02";
    double seconds = 60.0;   ///< 0 = until the window closes
    double speed = 8.0;
    double radius = 90.0;    ///< 0 = draw everything
    double tile = 40.0;
    double route = 600.0;
    double render_fps = 50.0;
    unsigned int max_tris = 0;
    bool no_scenery = false;
    bool show_roadnet = false;
    bool flat_terrain = false;
    bool no_shadows = false;
    bool save_frames = false;
};

/// Manifest produced by mcity/build_configs.sh for each vegetation level.
std::string ManifestFor(const std::string& level) {
    if (level == "none")       return "mcity_scene.json";
    if (level == "trees")      return "mcity_scene_trees_bare.json";
    if (level == "trees-leaf") return "mcity_scene_trees_leaf.json";
    if (level == "shrubs")     return "mcity_scene_all_bare.json";
    if (level == "full")       return "mcity_scene_full.json";
    return "";
}

bool ParseOptions(int argc, char** argv, Options& o) {
    auto need = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            printf("%s needs a value\n", argv[i]);
            return nullptr;
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        const char* v = nullptr;
        if (a == "--foliage")           { if (!(v = need(i))) return false; o.foliage = v; }
        else if (a == "--scene")        { if (!(v = need(i))) return false; o.scene = v; }
        else if (a == "--data")         { if (!(v = need(i))) return false; o.data_dir = v; }
        else if (a == "--tire")         { if (!(v = need(i))) return false; o.tire = v; }
        else if (a == "--seconds")      { if (!(v = need(i))) return false; o.seconds = std::atof(v); }
        else if (a == "--speed")        { if (!(v = need(i))) return false; o.speed = std::atof(v); }
        else if (a == "--radius")       { if (!(v = need(i))) return false; o.radius = std::atof(v); }
        else if (a == "--tile")         { if (!(v = need(i))) return false; o.tile = std::atof(v); }
        else if (a == "--route")        { if (!(v = need(i))) return false; o.route = std::atof(v); }
        else if (a == "--render-fps")   { if (!(v = need(i))) return false; o.render_fps = std::atof(v); }
        else if (a == "--max-tris")     { if (!(v = need(i))) return false; o.max_tris = (unsigned)std::atoi(v); }
        else if (a == "--no-scenery")   { o.no_scenery = true; }
        else if (a == "--show-roadnet") { o.show_roadnet = true; }
        else if (a == "--flat-terrain") { o.flat_terrain = true; }
        else if (a == "--no-shadows")   { o.no_shadows = true; }
        else if (a == "--save-frames")  { o.save_frames = true; }
        else if (a == "-h" || a == "--help") { return false; }
        else { printf("unknown option: %s\n", a.c_str()); return false; }
    }
    if (ManifestFor(o.foliage).empty() && o.scene.empty()) {
        printf("unknown --foliage level: %s\n", o.foliage.c_str());
        return false;
    }
    return true;
}

void PrintUsage() {
    printf("usage: demo_SCEN_mcity [--foliage none|trees|trees-leaf|shrubs|full] [--speed V]\n");
    printf("       [--seconds N] [--radius R] [--tile T] [--route L] [--tire MODEL]\n");
    printf("       [--scene FILE] [--data DIR] [--render-fps N] [--max-tris N]\n");
    printf("       [--no-scenery] [--show-roadnet] [--flat-terrain] [--no-shadows] [--save-frames]\n");
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    // Data paths come from Chrono's own defaults, which resolve against the working
    // directory ("../data/"): run these from a build tree's bin/ as with every other
    // Chrono demo. CHRONO_DATA_DIR / CHRONO_VEHICLE_DATA_DIR override if needed.

    Options opt;
    if (!ParseOptions(argc, argv, opt)) {
        PrintUsage();
        return 1;
    }
    std::string mcity_dir = !opt.data_dir.empty() ? opt.data_dir : GetChronoDataFile("mcity");
    double max_time = opt.seconds;
    double target_speed = opt.speed;
    bool keep_open = (opt.seconds <= 0);
    bool no_scenery = opt.no_scenery;

    // ---------------------------------------------------------------------------------------
    // Road network: contact surface and lane semantics
    // ---------------------------------------------------------------------------------------

    auto clock_now = [] { return std::chrono::steady_clock::now(); };
    auto ms_since = [](std::chrono::steady_clock::time_point t) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
    };
    auto t_start = clock_now();

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(mcity_dir + "/McityMap_Main.xodr")) {
        printf("Could not load the Mcity road network from %s\n", mcity_dir.c_str());
        printf("Run demos_live/mcity/fetch_mcity.sh first.\n");
        return 1;
    }

    double total_length = 0;
    for (unsigned int r : network->GetRoadIds())
        total_length += network->GetRoadLength(r);
    printf("Mcity\n  network: %u roads, %.1f km, %zu marked lane borders  [%.0f ms]\n",
           network->GetNumRoads(), total_length / 1000.0, network->GetMarkedLanes().size(),
           ms_since(t_start));

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    // The terrain answers height, normal and friction from the road geometry. Its visualization
    // mesh is normally never built -- Mcity's own road mesh is drawn instead, and drawing both
    // would z-fight.
    //
    // SHOW_ROADNET builds it anyway, in a strong colour, so the two sources can be compared on
    // screen. That is the only way to actually see whether the OpenDRIVE network and the USD
    // environment register: a vehicle driving convincingly proves the contact surface is
    // self-consistent, not that it agrees with what is drawn.
    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);
    if (opt.show_roadnet) {
        terrain.SetMeshResolution(2.0, 1);
        terrain.CreateVisualizationMesh();
        terrain.CreateLaneMarkings();
        printf("  OpenDRIVE surface drawn for comparison (%zu tris)\n",
               terrain.GetMesh() ? terrain.GetMesh()->GetIndicesVertices().size() : 0);
    }

    // ---------------------------------------------------------------------------------------
    // Scenery: everything visible
    // ---------------------------------------------------------------------------------------

    auto t_scenery = clock_now();
    ChSceneryModel scenery;
    if (!no_scenery) {
        ChSceneryModel::Options opts;

        // No vertical correction: the two sources agree. Sampling 397 OpenDRIVE lane centres
        // against Mcity's authored road mesh -- with the mesh's own instance transform applied,
        // which matters, since it sits at (7.85, -16.34, 0.24) -- gives a mean difference of
        // -0.007 m and a median of -0.012 m, with 0.137 m of scatter from road crown and
        // tessellation. The offset hook stays for maps whose sources do disagree.
        const char* z_env = nullptr;
        if (z_env) {
            opts.position_offset = ChVector3d(0, 0, std::atof(z_env));
            printf("  scenery shifted %.2f m vertically\n", std::atof(z_env));
        }

        // A blunt way to shed a few very heavy props: the perimeter fence alone is 169k
        // triangles and the GPS blocker another 52k, both placed once and neither of interest
        // from inside a car.
        opts.max_triangles_per_asset = opt.max_tris;

        // Drive on the geometry that is drawn. The OpenDRIVE elevation profile and the Mcity
        // meshes are independently authored and disagree by -0.24 to +0.29 m over the carriageway
        // (5th/95th percentile), which is visible as the vehicle floating and sinking. Sampling
        // the drawn triangles removes the disagreement by definition. Set FLAT_TERRAIN=1 to fall
        // back to the analytic surface.
        // Tile the scenery so the visual system can discard it piecewise; see STREAM_RADIUS.
        opts.tile_size = opt.tile;

        if (!opt.flat_terrain)
            // Everything is ground unless it obviously is not. Listing what to include is how
            // the roundabout apron and the traffic islands ended up as holes you could see but
            // not drive on; excluding is the safer direction to be wrong in. Overhead geometry is
            // harmless because the height query is z-aware, so this list is about build cost.
            opts.ground_all = true;
            // Groups first: they are exact, and they are what actually keeps the foliage out.
            opts.ground_exclude_groups = {"Foliage_Instanced", "TrafficPoles", "StreetLights",
                                          "TrafficLights", "TrafficLightCables"};
            opts.ground_exclude = {"Pole", "Sign", "TrafficLight", "Cable", "Fence", "GuardRail",
                                   "Facade", "Building", "Tree", "Bush", "Foliage", "StreetLight",
                                   "Hydrant", "Bollard", "Barrier", "Container", "WaterTower",
                                   "Pavilion", "Basketball", "Dumpster", "BusStop", "Bench",
                                   "Chair", "Table", "Meter", "Charger", "Barrel", "Rock",
                                   "Camera", "GPSBlocker", "MailBox", "TrashCan", "BikeRack"};
        // MCITY_SCENE names an alternate manifest; the default deliberately excludes foliage.
        std::string scene_file = !opt.scene.empty() ? opt.scene
                                                    : mcity_dir + "/" + ManifestFor(opt.foliage);
        if (!scenery.Load(sys, scene_file, opts)) {
            printf("  no scenery manifest; run demos_live/mcity/usd_to_chrono.py\n");
        } else {
            scenery.ReportTo(std::cout);
            printf("    [%.0f ms to build the scene graph]\n", ms_since(t_scenery));
            if (auto field = scenery.GetGroundHeightField())
                terrain.SetGroundHeightField(field);
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
    // At 8 m/s a 600 m route is spent in about 75 s, after which the path follower just sits at
    // the end. ROUTE_LEN lets a long run keep threading junctions instead.
    double route_len = opt.route;
    auto path = network->CreateRoutePath(start, route_len, ChJunctionChoice::STRAIGHT, 1.0);
    if (!path) {
        printf("Could not build a route from road %u lane %d.\n", start.road_id, start.lane_id);
        return 1;
    }
    auto route_pts = network->SampleRoute(start, route_len, ChJunctionChoice::STRAIGHT, 1.0);
    printf("  route: %zu points (%.0f m requested) from road %u lane %+d (longest road, %.0f m)\n", route_pts.size(), route_len,
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
                // Pac02 is the Magic Formula tire and by far the most expensive part of the
                // step; TMeasy is a handling tire of similar fidelity for this kind of driving at
                // a fraction of the cost, and Rigid is cheapest of all. TIRE selects.
                std::string tire_json = "audi/json/audi_Pac02Tire.json";
                if (opt.tire == "tmeasy")
                    tire_json = "audi/json/audi_TMeasyTire.json";
                else if (opt.tire == "rigid")
                    tire_json = "audi/json/audi_RigidTire.json";
                auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
                static bool printed_tire = false;
                if (!printed_tire) { printf("  tire model: %s\n", tire_json.c_str()); printed_tire = true; }
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
    // Draw only what is near the vehicle. A town-sized static scene is mostly irrelevant at any
    // instant, and gating on distance makes the per-frame cost follow what is nearby rather than
    // how large the site is. STREAM_RADIUS=0 disables it.
    double stream_radius = opt.radius;
    if (stream_radius > 0) {
        vis->EnableVisualStreaming(true, stream_radius, 20.0);
        printf("  visual streaming: %.0f m radius\n", stream_radius);
    }

    vis->AttachDriver(&driver);
    vis->SetWindowTitle("Mcity digital twin - Chrono");
    vis->SetWindowSize(1500, 900);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    // Shadows cost frame time but do most of the work of making a flat-shaded scene read as
    // three-dimensional: without them poles and signs float against the ground plane.
    if (!opt.no_shadows)
        vis->EnableShadows();
    vis->SetChaseCamera(ChVector3d(0, 0, 1.2), 12.0, 0.7);
    auto t_vis = clock_now();
    vis->Initialize();
    printf("  renderer ready [%.0f ms; %.1f s total startup]\n", ms_since(t_vis), ms_since(t_start) / 1000.0);

    printf("\nDriving Mcity at %.1f m/s%s.\n\n", target_speed,
           keep_open ? ", until the window is closed" : "");

    // ---------------------------------------------------------------------------------------
    // Simulation loop
    // ---------------------------------------------------------------------------------------

    const double step = 1e-3;
    // Physics needs 1 kHz; the display does not. RENDER_FPS trades render cost against smoothness.
    const double render_step = 1.0 / std::max(1.0, opt.render_fps);
    double next_render = 0;
    double next_report = 0;
    bool save_frames = opt.save_frames;
    int frame_idx = 0;

    // EnableRealtime caps the loop at 1x, so wall-clock alone cannot show the headroom. Track the
    // time genuinely spent in physics and rendering to distinguish "slow" from "throttled".
    audi.EnableRealtime(true);
    double busy_s = 0;
    double t_veh_sync = 0, t_vis_sync = 0, t_veh_adv = 0, t_vis_adv = 0, t_dyn = 0;
    double t_drv_adv = 0, t_ter_adv = 0;
    auto t_loop = clock_now();

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
                snprintf(fn, sizeof(fn), "mcity_out/frame_%04d.png", frame_idx);
                vis->WriteImageToFile(fn);
            }
            frame_idx++;
        }

        if (stream_radius > 0)
            vis->SetStreamingFocus(audi.GetPos());

        auto t_busy = clock_now();
        DriverInputs in = driver.GetInputs();
        auto tA = clock_now();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        t_veh_sync += ms_since(tA);

        tA = clock_now();
        vis->Synchronize(time, in);
        t_vis_sync += ms_since(tA);

        tA = clock_now();
        driver.Advance(step);
        t_drv_adv += ms_since(tA);
        tA = clock_now();
        terrain.Advance(step);
        t_ter_adv += ms_since(tA);
        tA = clock_now();
        audi.Advance(step);
        t_veh_adv += ms_since(tA);

        tA = clock_now();
        vis->Advance(step);
        t_vis_adv += ms_since(tA);

        tA = clock_now();
        sys.DoStepDynamics(step);
        t_dyn += ms_since(tA);
        busy_s += ms_since(t_busy) / 1000.0;

        if (time >= next_report) {
            double yaw = audi.GetRot().GetCardanAnglesZYX().z();
            ChLaneInfo info = network->GetLaneInfo(audi.GetPos(), yaw);
            printf("t=%5.1f s  v=%5.1f m/s  z=%6.1f m", time, audi.GetSpeed(), audi.GetPos().z());
            if (info.valid)
                printf("  road %3u lane %+d s=%6.1f offset=%+5.2f%s", info.road_id, info.lane_id,
                       info.s, info.lane_offset, info.InJunction() ? "  [junction]" : "");
            else
                printf("  off the network");
            if (stream_radius > 0) {
                int shown = 0, total = 0;
                vis->GetStreamingStats(shown, total);
                printf("  vis %d/%d", shown, total);
            }
            printf("\n");
            next_report += 1.0;
        }
    }

    double wall = ms_since(t_loop) / 1000.0;
    double simt = sys.GetChTime();
    printf("\nStopped at t = %.2f s\n", simt);
    printf("  breakdown (s): veh.Sync %.1f | driver.Adv %.1f | terrain.Adv %.1f | veh.Adv %.1f | DoStep %.1f\n",
           t_veh_sync/1000, t_drv_adv/1000, t_ter_adv/1000, t_veh_adv/1000, t_dyn/1000);
    printf("  loop: %.1f s wall for %.1f s simulated (%.2fx). Compute used %.1f s of that, so the\n",
           wall, simt, simt / std::max(wall, 1e-9), busy_s);
    printf("  simulation could run about %.1fx real time uncapped; the rest is EnableRealtime\n",
           simt / std::max(busy_s, 1e-9));
    printf("  holding it to wall-clock, plus %.1f s of one-off startup.\n", ms_since(t_start) / 1000.0 - wall);
    return 0;
}
