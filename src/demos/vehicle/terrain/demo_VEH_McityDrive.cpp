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
// Drive an Audi around the Mcity digital twin under keyboard control.
//
// -----------------------------------------------------------------------------
// GETTING THE SCENE
// -----------------------------------------------------------------------------
// Chrono ships the converter, not the scene: the Mcity assets are a third-party
// dataset of a few hundred megabytes under their own licence. Build it once,
// from this directory:
//
//     cd src/demos/vehicle/terrain/mcity
//     python3 -m pip install usd-core        # one-off, the converter reads USD
//     ./setup_mcity.sh --repo /path/to/mcity-digital-twin
//
// Omit --repo to fetch over HTTPS instead. Add --foliage for vegetation, which
// downloads a further ~200 MB and builds the --foliage levels below.
//
//     git clone https://github.com/mcity/mcity-digital-twin
//
// Output lands in <chrono>/data/mcity and is entirely generated: delete it and
// re-run to rebuild. Then, from a build tree:
//
//     cd bin && ./demo_VEH_McityDrive
//
// -----------------------------------------------------------------------------
// WHAT THIS USES
// -----------------------------------------------------------------------------
// Chrono::Vehicle and Chrono::VSG. There is no road network here and no
// OpenDRIVE: the scene is a set of meshes with placements, and the ground is a
// merged mesh of those same surfaces handed to a stock RigidTerrain.
//
// The only new piece is ChSceneryModel, which loads a placement manifest -- a
// few hundred meshes instanced a few thousand times, one visual shape per mesh
// however often it appears. The converter writes both that manifest and the
// merged ground mesh, so nothing about Mcity is compiled into Chrono.
//
// Driving on the drawn geometry matters here rather than being a nicety. Mcity
// also publishes an OpenDRIVE network, and its elevation profile differs from
// the artist's road mesh by -0.24 to +0.29 m at the 5th and 95th percentiles --
// enough to watch the vehicle float and sink. Using the meshes for both makes
// the two the same surface by construction.
//
// Controls: W/S throttle and brake, A/D steer, plus the usual VSG camera keys.
//
// =============================================================================

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChSceneryModel.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

// A pose on a real Mcity lane, read once from the published OpenDRIVE network so that this demo
// does not have to. Road 29, right-hand lane, facing along the carriageway.
const double kStartX = 158.923;
const double kStartY = 62.991;
const double kStartYaw = 1.285431;

// Vegetation levels, as written by mcity/build_configs.sh. Only "none" exists unless setup_mcity.sh
// was run with --foliage.
std::string ManifestFor(const std::string& level) {
    if (level == "none") return "mcity_scene.json";
    if (level == "trees") return "mcity_scene_trees_bare.json";
    if (level == "trees-leaf") return "mcity_scene_trees_leaf.json";
    if (level == "shrubs") return "mcity_scene_all_bare.json";
    if (level == "full") return "mcity_scene_full.json";
    return "";
}

void PrintUsage() {
    printf(
        "\n"
        "demo_VEH_McityDrive -- drive an Audi around the Mcity digital twin.\n"
        "\n"
        "  W/S  throttle and brake      A/D  steer\n"
        "\n"
        "Options\n"
        "  --foliage LEVEL   none | trees | trees-leaf | shrubs | full   (default none)\n"
        "                      none        no vegetation\n"
        "                      trees       447 trees, bare branches\n"
        "                      trees-leaf  447 trees with leaves\n"
        "                      shrubs      trees and shrubs, bare branches\n"
        "                      full        everything with leaves (heavy)\n"
        "                    Anything but 'none' needs setup_mcity.sh --foliage.\n"
        "  --data DIR        converted scene directory (default <chrono data>/mcity)\n"
        "  --speed-limit V   speed the throttle is scaled toward, m/s (default 20)\n"
        "  --tire MODEL      pac02 | tmeasy | rigid   (default pac02)\n"
        "  --tire-step S     tire internal step, s (default 1e-4)\n"
        "                             build at startup and roughly half the frame rate.\n"
        "  -h, --help        this message\n"
        "\n"
        "First time? The scene is generated, not shipped:\n"
        "  cd src/demos/vehicle/terrain/mcity\n"
        "  python3 -m pip install usd-core\n"
        "  ./setup_mcity.sh --repo /path/to/mcity-digital-twin     # or omit --repo to download\n"
        "  git clone https://github.com/mcity/mcity-digital-twin   # if you need a clone\n"
        "\n");
}

int main(int argc, char** argv) {
    // Line-buffered: this demo reports progress, and a redirected stdout would otherwise hold it
    // all until exit -- which makes a slow startup look like a hang.
    setvbuf(stdout, nullptr, _IOLBF, 0);

    std::string foliage = "none";
    std::string data_dir;
    double speed_limit = 20.0;
    std::string tire = "pac02";
    double tire_step = 1e-4;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--foliage" && i + 1 < argc)
            foliage = argv[++i];
        else if (a == "--data" && i + 1 < argc)
            data_dir = argv[++i];
        else if (a == "--speed-limit" && i + 1 < argc)
            speed_limit = std::atof(argv[++i]);
        else if (a == "--tire" && i + 1 < argc)
            tire = argv[++i];
        else if (a == "--tire-step" && i + 1 < argc)
            tire_step = std::atof(argv[++i]);
        else {
            PrintUsage();
            return (a == "-h" || a == "--help") ? 0 : 1;
        }
    }
    if (ManifestFor(foliage).empty()) {
        printf("unknown --foliage level: %s\n", foliage.c_str());
        PrintUsage();
        return 1;
    }

    if (tire != "pac02" && tire != "tmeasy" && tire != "rigid") {
        printf("unknown --tire model: %s\n", tire.c_str());
        PrintUsage();
        return 1;
    }

    std::string mcity_dir = data_dir.empty() ? GetChronoDataFile("mcity") : data_dir;
    std::string manifest = mcity_dir + "/" + ManifestFor(foliage);
    std::string ground_obj = mcity_dir + "/mcity_ground.obj";

    auto t_boot = std::chrono::steady_clock::now();

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver settings, matching what Chrono's own road-driving demos use. The defaults are a
    // projected Gauss-Seidel solver with few iterations, which under-solves a vehicle's
    // suspension constraints and shows up as the suspension juddering for no visible reason.
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // ---------------------------------------------------------------------------------------
    // Scene
    // ---------------------------------------------------------------------------------------

    ChSceneryModel scenery;
    if (!scenery.Load(sys, manifest, ChSceneryOptions())) {
        printf("\nCould not read %s\n", manifest.c_str());
        PrintUsage();
        return 1;
    }
    scenery.ReportTo(std::cout);

    // ---------------------------------------------------------------------------------------
    // Ground
    //
    // One merged mesh of every drivable surface, written by the converter. Visualization is off:
    // the scenery already draws these triangles, and drawing them twice would z-fight.
    // ---------------------------------------------------------------------------------------

    std::ifstream gf(ground_obj);
    if (!gf.good()) {
        printf("\nCould not read %s\n", ground_obj.c_str());
        printf("Re-run setup_mcity.sh; the converter writes it alongside the manifest.\n");
        return 1;
    }

    // Contact material, matching the tuning used by the Chrono HIL teleop scenes: friction 0.9,
    // restitution 0.01, and a Young's modulus of 2e7. The stiffness matters -- the default is far
    // stiffer than a road needs and makes the solver work harder for no benefit.
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto ground_mat = minfo.CreateMaterial(sys.GetContactMethod());

    RigidTerrain terrain(&sys);
    // connected_mesh = false: the collision BVH indexes triangles and does not need vertex
    // adjacency. Visualization off: the scenery already draws these triangles.
    terrain.AddPatch(ground_mat, CSYSNORM, ground_obj, false, 0, false);
    terrain.Initialize();

    // ---------------------------------------------------------------------------------------
    // Vehicle
    // ---------------------------------------------------------------------------------------

    // For RigidTerrain the spawn height comes from the mesh file, not from the terrain.
    //
    // RigidTerrain answers height queries by raycasting the collision system, and that system is
    // not built until the first DoStepDynamics. Asking during setup misses, and GetHeight returns
    // its miss value of zero -- which on a site whose datum is 274 m drops the vehicle out of the
    // world. Reading the ground mesh directly avoids the ordering entirely.
    double ground_z = 274.26;
    {
        std::ifstream f(ground_obj);
        std::string tag;
        double x, y, z, best = -1e30;
        while (f >> tag) {
            if (tag != "v") {
                std::getline(f, tag);
                continue;
            }
            f >> x >> y >> z;
            if (std::abs(x - kStartX) < 2.0 && std::abs(y - kStartY) < 2.0 && z > best)
                best = z;
        }
        ground_z = (best > -1e29) ? best : 274.26;
    }

    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(ChCoordsys<>(ChVector3d(kStartX, kStartY, ground_z + 0.5), QuatFromAngleZ(kStartYaw)));
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
            std::string tire_json = "audi/json/audi_Pac02Tire.json";
            if (tire == "tmeasy")
                tire_json = "audi/json/audi_TMeasyTire.json";
            else if (tire == "rigid")
                tire_json = "audi/json/audi_RigidTire.json";
            auto t = ReadTireJSON(GetVehicleDataFile(tire_json));
            t->SetStepsize(tire_step);
            audi.InitializeTire(t, wheel, VisualizationType::MESH);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Keyboard control and rendering
    // ---------------------------------------------------------------------------------------

    ChInteractiveDriver driver(audi);
    driver.SetSteeringDelta(0.04);
    driver.SetThrottleDelta(1.0 / std::max(1.0, speed_limit));
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

    printf("\n  [%.1f s to build the scene and open the window]\n", 
           std::chrono::duration<double>(std::chrono::steady_clock::now() - t_boot).count());
    printf("W/S throttle and brake, A/D steer.\n\n");
    auto t_loop = std::chrono::steady_clock::now();
    double next_report = 0;
    double render_s = 0, physics_s = 0;

    const double step = 1e-3;
    const double render_step = 1.0 / 50;  // physics wants 1 kHz; the display does not
    double next_render = 0;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        double time = sys.GetChTime();

        if (time >= next_render) {
            auto tR = std::chrono::steady_clock::now();
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - tR).count();
            next_render += render_step;
        }


        if (time >= next_report) {
            double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_loop).count();
            printf("  t=%5.1f s   %.2fx real time   (physics %.1f s, render %.1f s of %.1f s wall)\n",
                   time, wall > 0 ? time / wall : 0, physics_s, render_s, wall);
            next_report += 2.0;
        }

        auto tP = std::chrono::steady_clock::now();
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
        physics_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - tP).count();

        // Throttle once per rendered frame, not once per physics step.
        //
        // Spinning every step caps each one at the step size, so the 19 steps between frames
        // sleep away their slack and the 20th still has to pay for the render. The loop can
        // never make that back, and a scene that runs comfortably faster than real time reports
        // roughly half speed. Pacing per frame lets the fast steps absorb the slow one.
        if (time >= next_render - step)
            realtime.Spin(render_step);
    }

    return 0;
}
