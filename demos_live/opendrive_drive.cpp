// An Audi driving an ASAM OpenDRIVE road network, with the road surface supplied by
// ChOpenDriveTerrain and steering by a ChPathFollowerDriver tracking a lane center line
// extracted from the network. Metal RT chase camera.
//
// The point of this demo is the *addressing*. Everything placed in the scene -- the vehicle
// spawn, the lane the driver follows, the cone taper -- is expressed in lane coordinates
// (road, lane, s, lateral offset) rather than hand-computed world offsets. Compare
// openpilot_drive.cpp, which had to write `spawn.y() - 1.8 + f * 3.0` because no lane existed
// to reference.
//
// Usage:  ./opendrive_drive [path/to/road.xodr] [spawn_s]
//
// Defaults to esmini's curve_r100.xodr, starting at s = 480 m. That road runs straight for its
// first ~500 m and then bends at r = 100 m, so starting there puts both the lane tracking and the
// cone taper on the curve -- which is where world-frame offsets would have gone wrong.
// Override the esmini resource root with ESMINI_ROOT.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_scenario/ChOpenDriveNetwork.h"
#include "chrono_scenario/ChOpenDriveTerrain.h"

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
}  // namespace

int main(int argc, char** argv) {
    // Line-buffer stdout so the per-second telemetry still appears when output is piped to a file.
    setvbuf(stdout, nullptr, _IOLBF, 0);

    SetChronoDataPath(std::string(kChronoRoot) + "data/");
    vehicle::SetVehicleDataPath(std::string(kChronoRoot) + "data/vehicle/");

    const char* esmini_root_env = std::getenv("ESMINI_ROOT");
    std::string esmini_root = esmini_root_env ? esmini_root_env : kEsminiRootDefault;
    std::string xodr_file = (argc > 1) ? argv[1] : esmini_root + "/resources/xodr/curve_r100.xodr";

    // ---------------------------------------------------------------------------------------
    // Road network
    // ---------------------------------------------------------------------------------------

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(xodr_file)) {
        printf("Could not load OpenDRIVE file: %s\n", xodr_file.c_str());
        return 1;
    }

    auto road_ids = network->GetRoadIds();
    if (road_ids.empty()) {
        printf("Network contains no roads.\n");
        return 1;
    }

    unsigned int road_id = road_ids[0];
    double road_length = network->GetRoadLength(road_id);

    auto lane_ids = network->GetLaneIds(road_id, 0.0);
    if (lane_ids.empty()) {
        printf("Road %u has no drivable lanes at s = 0.\n", road_id);
        return 1;
    }

    // Prefer a right-hand lane (negative ID) -- the usual travel direction for these samples.
    int lane_id = lane_ids[0];
    for (int id : lane_ids) {
        if (id < 0) {
            lane_id = id;
            break;
        }
    }

    printf("Loaded %s\n", xodr_file.c_str());
    printf("  %u road(s); driving road %u, lane %d, length %.1f m, lane width %.2f m\n",
           network->GetNumRoads(), road_id, lane_id, road_length,
           network->GetLaneWidth(road_id, lane_id, 0.0));

    // ---------------------------------------------------------------------------------------
    // System, terrain, vehicle
    // ---------------------------------------------------------------------------------------

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);
    // As in audi_drive.cpp: the default 50 NSC iterations let the double-wishbone suspension
    // camber-buckle under torque transients.
    if (sys.GetSolver() && sys.GetSolver()->AsIterative())
        sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetMeshResolution(1.0, 4);
    terrain.SetRoadDiffuseTextureFile(GetVehicleDataFile("terrain/textures/concrete.jpg"), 0.35f, 0.35f);
    terrain.CreateVisualizationMesh();
    // Painted lane lines, read from the file's OpenDRIVE <roadMark> entries. These are what a
    // lane-detection model actually keys on, so a road without them is not a fair test of one.
    terrain.CreateLaneMarkings();

    // Spawn in lane coordinates rather than world coordinates.
    double spawn_s = (argc > 2) ? std::atof(argv[2]) : 480.0;
    spawn_s = std::min(spawn_s, std::max(0.0, road_length - 100.0));

    ChCoordsys<> spawn = network->LaneToWorld({road_id, lane_id, spawn_s, 0.0}, false);
    spawn.pos.z() += 0.65;  // start above the road; settles to ride height

    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(spawn);
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

    // ---------------------------------------------------------------------------------------
    // Driver: follow the lane center line straight out of the network
    // ---------------------------------------------------------------------------------------

    auto path = network->CreateLaneCenterPath(road_id, lane_id, 1.0);
    if (!path) {
        printf("Could not extract a center line for road %u lane %d.\n", road_id, lane_id);
        return 1;
    }

    const double target_speed = 12.0;  // m/s
    ChPathFollowerDriver driver(audi, path, "lane_center", target_speed, 0.5, 2.0);
    driver.GetSteeringController().SetLookAheadDistance(8.0);
    driver.GetSteeringController().SetGains(0.6, 0.0, 0.0);
    // Small Ki: the throttle ramp otherwise winds the integrator up and overshoots the target.
    driver.GetSpeedController().SetGains(0.4, 0.01, 0.0);
    driver.Initialize();

    // ---------------------------------------------------------------------------------------
    // Cone taper, addressed in lane coordinates
    //
    // The taper walks from the lane center to the right lane edge over its length. Because the
    // offsets are lane-relative, this is correct on a curve, on a banked section, and on any
    // other road in the network -- no world-frame arithmetic and no assumption that +y is left.
    // ---------------------------------------------------------------------------------------

    {
        const int num_cones = 12;
        const double taper_start_s = spawn_s + 80.0;
        const double cone_spacing = 3.0;

        auto orange = chrono_types::make_shared<ChVisualMaterial>();
        orange->SetDiffuseColor(ChColor(0.95f, 0.32f, 0.05f));

        int placed = 0;
        for (int i = 0; i < num_cones; i++) {
            double s = taper_start_s + i * cone_spacing;
            if (s > road_length)
                break;

            double half_width = 0.5 * network->GetLaneWidth(road_id, lane_id, s);
            double frac = i / double(num_cones - 1);
            // Negative offset is right of the lane center, so this closes the lane from the right.
            double offset = -frac * half_width;

            ChCoordsys<> cone_pose;
            if (!network->LaneToWorld({road_id, lane_id, s, offset}, cone_pose, false))
                continue;

            auto cone = chrono_types::make_shared<ChBody>();
            auto shape = chrono_types::make_shared<ChVisualShapeModelFile>();
            shape->SetFilename(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj"));
            cone->AddVisualShape(shape, ChFrame<double>(ChVector3d(0, 0, 0), QUNIT));
            cone->SetFixed(true);
            cone->SetPos(cone_pose.pos);

            if (auto vm = cone->GetVisualModel())
                for (auto& si : vm->GetShapeInstances())
                    for (int m = 0; m < std::max(1, (int)si.shape->GetMaterials().size()); m++)
                        si.shape->SetMaterial(m, orange);

            sys.Add(cone);
            placed++;
        }
        printf("  placed %d cones tapering to the right lane edge from s = %.1f m\n", placed,
               taper_start_s);
    }

    // ---------------------------------------------------------------------------------------
    // Sensors
    // ---------------------------------------------------------------------------------------

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    // OpenDRIVE supplies no surroundings at all, so without an environment light the road sits in
    // the dark against pure black.
    manager->scene->AddEnvironmentLight(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));
    manager->scene->SetAmbientLight(ChVector3f(0.25f, 0.25f, 0.28f));
    manager->scene->AddPointLight(ChVector3f(20, 10, 25), ChColor(1.0f, 0.98f, 0.92f), 200.f);
    manager->scene->AddPointLight(ChVector3f(-20, -15, 20), ChColor(0.35f, 0.4f, 0.55f), 150.f);

    ChVector3d cam_off(-8.0, 0.0, 2.8);
    ChVector3d look(4.0, 0.0, 0.5);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off,
                             QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 60.0f, cam_pose, 1280, 720,
                                                         (float)(CH_PI / 3), 2);
    cam->SetName("opendrive_chase");
    // Opt-in frame capture: set SAVE_FRAMES to leave a visual record of a run.
    if (std::getenv("SAVE_FRAMES"))
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(std::string(kChronoRoot) +
                                                                "demos_live/opendrive_out/"));
    auto vis = chrono_types::make_shared<ChFilterMetalVisualize>(1280, 720, "OpenDRIVE - chase (Metal RT)");
    cam->PushFilter(vis);
    manager->AddSensor(cam);

    // ---------------------------------------------------------------------------------------
    // Simulation loop
    // ---------------------------------------------------------------------------------------

    printf("\nDriving the lane center line - close the window to stop.\n\n");

    const double step = 1e-3;
    double time = 0;
    double next_report = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (vis->WindowOpen()) {
        DriverInputs in = driver.GetInputs();

        driver.Synchronize(time);
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);

        driver.Advance(step);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();

        // Lane-referenced telemetry: where the vehicle is in terms the road defines, which is
        // what a scenario metric or a lane-referenced label would be written against.
        if (time >= next_report) {
            // The chassis reference frame, not GetChassisBody()->GetPos(), which is the center of
            // mass. They differ only vertically for this vehicle, but the reference frame is the
            // frame Chrono placed and the one worth being consistent about.
            double yaw = audi.GetRot().GetCardanAnglesZYX().z();
            ChLaneInfo info = network->GetLaneInfo(audi.GetPos(), yaw);

            if (info.valid) {
                printf("t=%5.1f s  lane %+d  s=%6.1f m  offset=%+5.2f m  width=%.2f m  "
                       "curv=%+.4f 1/m  v=%4.1f m/s%s\n",
                       time, info.lane_id, info.s, info.lane_offset, info.lane_width, info.curvature,
                       audi.GetSpeed(), info.InJunction() ? "  [junction]" : "");
            } else {
                printf("t=%5.1f s  off the road network  v=%4.1f m/s\n", time, audi.GetSpeed());
            }
            next_report += 1.0;
        }

        time += step;
        double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        if (time > wall)
            std::this_thread::sleep_for(std::chrono::duration<double>(time - wall));
    }

    printf("\nStopped at t = %.2f s\n", time);
    return 0;
}
