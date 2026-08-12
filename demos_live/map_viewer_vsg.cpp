// Viewer for an ASAM OpenDRIVE road network, drawn with Chrono::VSG.
//
// Chrono has run-time visualizers for vehicles and test rigs, but nothing that simply shows a
// road network. This loads any .xodr, builds the driving surface and its painted lane markings,
// and reports what came out -- roads, junctions, lane counts, marked borders and mesh size --
// which together are enough to tell whether a third-party map imported correctly.
//
// Usage:  ./map_viewer_vsg <road.xodr> [seconds]
// Env:    KEEP_OPEN=1  ignore the time limit and run until the window is closed
//
// Drag to orbit, scroll to zoom. The camera is framed on the network's own extent, so this works
// on anything from a 750 m test road to a full facility map.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_scenario/ChOpenDriveNetwork.h"
#include "chrono_scenario/ChOpenDriveTerrain.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::scenario;
using namespace chrono::vsg3d;

namespace {
const char* kChronoRoot = "/Users/kylesha/Documents/sbel/chrono-sensor-metal/";
}

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2) {
        printf("usage: map_viewer_vsg <road.xodr> [seconds]\n");
        return 1;
    }

    SetChronoDataPath(std::string(kChronoRoot) + "data/");
    vehicle::SetVehicleDataPath(std::string(kChronoRoot) + "data/vehicle/");

    std::string xodr_file = argv[1];
    double max_time = (argc > 2) ? std::atof(argv[2]) : 60.0;
    bool keep_open = std::getenv("KEEP_OPEN") != nullptr;

    ChSystemNSC sys;

    auto network = chrono_types::make_shared<ChOpenDriveNetwork>();
    if (!network->Initialize(xodr_file)) {
        printf("Could not load %s\n", xodr_file.c_str());
        return 1;
    }

    ChOpenDriveTerrain terrain(&sys, network);
    terrain.SetMeshResolution(2.0, 1);
    terrain.SetRoadDiffuseTextureFile(vehicle::GetVehicleDataFile("terrain/textures/concrete.jpg"),
                                      0.35f, 0.35f);
    terrain.CreateVisualizationMesh();
    terrain.CreateLaneMarkings();

    auto road_mesh = terrain.GetMesh();
    if (!road_mesh || road_mesh->GetCoordsVertices().empty()) {
        printf("The network produced no surface geometry.\n");
        return 1;
    }

    // Summarise the import, then frame the camera on whatever came out. Both come from the mesh
    // itself rather than assumed map bounds, so an unfamiliar map needs no hand-tuning.
    auto roads = network->GetRoadIds();
    int lane_count = 0;
    double total_length = 0;
    for (unsigned int r : roads) {
        total_length += network->GetRoadLength(r);
        lane_count += static_cast<int>(network->GetLaneIds(r, 0.0).size());
    }

    ChVector3d lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (const auto& p : road_mesh->GetCoordsVertices()) {
        lo = ChVector3d(std::min(lo.x(), p.x()), std::min(lo.y(), p.y()), std::min(lo.z(), p.z()));
        hi = ChVector3d(std::max(hi.x(), p.x()), std::max(hi.y(), p.y()), std::max(hi.z(), p.z()));
    }
    ChVector3d center = 0.5 * (lo + hi);
    double span = std::max(hi.x() - lo.x(), hi.y() - lo.y());

    auto marking_mesh = terrain.GetLaneMarkingMesh();
    printf("\n%s\n", xodr_file.c_str());
    printf("  roads %zu | drivable lanes at s=0 %d | total length %.0f m (%.1f km)\n", roads.size(),
           lane_count, total_length, total_length / 1000.0);
    printf("  marked lane borders %zu\n", network->GetMarkedLanes().size());
    printf("  surface %zu tris | markings %zu tris\n", road_mesh->GetIndicesVertices().size(),
           marking_mesh ? marking_mesh->GetIndicesVertices().size() : 0);
    printf("  extent %.0f x %.0f m, elevation %.1f to %.1f m\n\n", hi.x() - lo.x(), hi.y() - lo.y(),
           lo.z(), hi.z());

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("OpenDRIVE map viewer");
    vis->SetWindowSize(1400, 900);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    // Back off by the network's own span so the whole map is in frame regardless of its size.
    vis->AddCamera(center + ChVector3d(-0.75 * span, -0.75 * span, 0.7 * span), center);
    vis->Initialize();

    printf("Drag to orbit, scroll to zoom.%s\n\n", keep_open ? " Runs until the window is closed." : "");

    // Nothing here is dynamic, so the system is never stepped; this is a viewer, not a simulation.
    double time = 0;
    const double frame_step = 1.0 / 50;
    while (vis->Run()) {
        if (!keep_open && time >= max_time)
            break;
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        time += frame_step;
    }

    return 0;
}
