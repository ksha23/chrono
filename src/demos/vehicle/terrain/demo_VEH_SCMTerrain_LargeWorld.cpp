// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Wheeled vehicle on an SCM patch far larger than would otherwise fit in memory.
//
// SCM normally builds one visualization mesh covering the entire declared extent, at grid resolution,
// allocated up front whether or not the terrain is ever touched. That costs O(extent / delta^2) and puts
// a kilometre-scale patch out of reach: 1 km x 1 km at 5 cm is 400 million vertices, tens of GiB.
//
// SetVisualizationWindow() instead keeps a fixed pool of mesh tiles and moves them to follow the active
// domains, so visualization memory depends on the window and not on the extent. Deformation itself is
// stored sparsely and is unaffected -- it is exact everywhere, for the whole run, window or no window.
//
// Run with --window 0 to get the old behaviour and watch the memory figure track the terrain size.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#ifdef __linux__
    #include <unistd.h>
#endif

#include "chrono/core/ChTypes.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// Resident set size in MiB. Only meaningful on Linux; elsewhere the demo still runs, it just cannot
// report the number that makes the point.
static double RssMiB() {
#ifdef __linux__
    std::ifstream f("/proc/self/statm");
    long total = 0, resident = 0;
    if (f >> total >> resident)
        return resident * (double)sysconf(_SC_PAGESIZE) / (1024.0 * 1024.0);
#endif
    return -1;
}

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    ChCLI cli(argv[0], "Wheeled vehicle on a large SCM terrain patch");
    cli.AddOption<double>("Demo", "s,size", "Terrain extent (square, m)", "1000");
    cli.AddOption<double>("Demo", "w,window", "Visualization window (m); 0 disables windowing", "40");
    cli.AddOption<double>("Demo", "d,delta", "SCM grid spacing (m)", "0.05");
    cli.AddOption<double>("Demo", "t,time", "Simulation length (s)", "20");
    cli.AddOption<double>("Demo", "speed", "Target speed (m/s)", "6");
    cli.AddOption<int>("Demo", "tile", "Grid cells per visualization tile edge", "64");
    cli.AddOption<bool>("Demo", "v,vis", "Enable run-time visualization", "false");

    if (!cli.Parse(argc, argv, true))
        return 0;

    const double terrain_size = cli.GetAsType<double>("size");
    const double window = cli.GetAsType<double>("window");
    const double delta = cli.GetAsType<double>("delta");
    const double t_end = cli.GetAsType<double>("time");
    const double target_speed = cli.GetAsType<double>("speed");
    const int tile_cells = cli.GetAsType<int>("tile");
    const bool visualize = cli.GetAsType<bool>("vis");

    const double step_size = 2e-3;

    // The declared extent in grid nodes. Without a window this is exactly how many mesh vertices SCM would
    // have to allocate, which is the whole point of the demo.
    const long nv = 2 * (long)std::ceil((terrain_size / 2) / delta) + 1;
    cout << "\nTerrain:  " << terrain_size << " x " << terrain_size << " m at " << delta << " m spacing" << endl;
    cout << "Grid:     " << nv << " x " << nv << " nodes  (" << (double)nv * nv / 1e6 << " M)" << endl;
    if (window > 0)
        cout << "Window:   " << window << " x " << window << " m, " << tile_cells << " cells/tile" << endl;
    else
        cout << "Window:   disabled (single full-extent mesh)" << endl;
    cout << endl;

    // ------------------
    // Create the vehicle
    // ------------------

    HMMWV_Full hmmwv;
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-terrain_size / 2 + 5, 0, 0.7), QUNIT));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    hmmwv.GetSystem()->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));

    // ------------------
    // Create the terrain
    // ------------------

    const double rss_before = RssMiB();
    const auto t0 = std::chrono::high_resolution_clock::now();

    SCMTerrain terrain(hmmwv.GetSystem());
    terrain.SetSoilParameters(2e6,    // Bekker Kphi
                              0,      // Bekker Kc
                              1.1,    // Bekker n
                              0,      // Mohr cohesion (Pa)
                              30,     // Mohr friction (degrees)
                              0.01,   // Janosi shear (m)
                              4e7,    // elastic stiffness (Pa/m)
                              3e4);   // damping (Pa s/m)
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);

    // Restrict ray casting to the neighbourhood of each wheel. This is what bounds the per-step cost;
    // it is independent of the visualization window, which bounds memory.
    for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
        terrain.AddActiveDomain(axle->GetWheel(VehicleSide::LEFT)->GetSpindle(), ChVector3d(0, 0, 0),
                                ChVector3d(1.0, 0.6, 1.0));
        terrain.AddActiveDomain(axle->GetWheel(VehicleSide::RIGHT)->GetSpindle(), ChVector3d(0, 0, 0),
                                ChVector3d(1.0, 0.6, 1.0));
    }

    if (window > 0)
        terrain.SetVisualizationWindow(window, window, tile_cells, true);

    terrain.Initialize(terrain_size, terrain_size, delta);

    const auto t1 = std::chrono::high_resolution_clock::now();
    const double init_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double rss_after = RssMiB();

    cout << "Terrain init:  " << init_ms << " ms";
    if (rss_before >= 0)
        cout << ",  resident memory +" << (rss_after - rss_before) << " MiB";
    cout << endl;
    cout << "Vis meshes:    " << terrain.GetMeshes().size() << endl << endl;

    // -----------------
    // Create the driver
    // -----------------

    auto path = StraightLinePath(ChVector3d(-terrain_size / 2 + 5, 0, 0.5),  //
                                 ChVector3d(+terrain_size / 2 - 5, 0, 0.5), 1);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "straight", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ------------------------
    // Run-time visualization
    // ------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    if (visualize) {
#ifdef CHRONO_IRRLICHT
        auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vis_irr->SetWindowTitle("SCM large world");
        vis_irr->SetChaseCamera(ChVector3d(0, 0, 0), 8.0, 1.0);
        vis_irr->Initialize();
        vis_irr->AddLightDirectional();
        vis_irr->AddSkyBox();
        vis_irr->AddLogo();
        vis_irr->AttachVehicle(&hmmwv.GetVehicle());
        vis = vis_irr;
#else
        cout << "Run-time visualization requested but Chrono::Irrlicht is not enabled; running headless."
             << endl;
#endif
    }

    // ---------------
    // Simulation loop
    // ---------------

    const int n_steps = (int)std::ceil(t_end / step_size);
    double t_scm = 0;
    double peak_rss = rss_after;

    const auto ts0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < n_steps; i++) {
        const double time = hmmwv.GetSystem()->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        DriverInputs driver_inputs = driver.GetInputs();

        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        t_scm += terrain.GetTimerActiveDomains() + terrain.GetTimerRayCasting() +
                 terrain.GetTimerContactPatches() + terrain.GetTimerContactForces() +
                 terrain.GetTimerVisUpdate();

        if (i % 500 == 0) {
            peak_rss = std::max(peak_rss, RssMiB());
            const auto& p = hmmwv.GetVehicle().GetPos();
            cout << "t = " << time << " s   x = " << p.x() << " m";
            if (rss_before >= 0)
                cout << "   resident " << RssMiB() << " MiB";
            cout << endl;
        }
    }
    const auto ts1 = std::chrono::high_resolution_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(ts1 - ts0).count();

    const auto& pos = hmmwv.GetVehicle().GetPos();
    cout << "\n--- summary ---" << endl;
    cout << "Distance travelled:  " << (pos.x() + terrain_size / 2 - 5) << " m" << endl;
    cout << "Wall time per step:  " << total_ms / n_steps << " ms" << endl;
    cout << "  of which SCM:      " << t_scm / n_steps << " ms" << endl;
    if (rss_before >= 0)
        cout << "Peak resident:       " << peak_rss << " MiB" << endl;
    terrain.PrintStepStatistics(cout);

    return 0;
}
