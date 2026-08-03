// SHOWCASE: Chrono::Sensor Metal RT backend -- Depth camera (ChDepthCamera).
// A stationary Audi on flat terrain under the shipped sky_2_4k HDR environment map. The depth camera orbits
// the car for 150 frames (full 360 deg) and saves each colorized depth frame as a PNG, then returns. HEADLESS.
// PNGs -> demos_live/showcase_out/camera_depth/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

int main(int argc, char** argv) {
    // Data root: $CHRONO_ROOT if set, else the repo this demo was built from (see tools/README.md).
    const char* env_root = std::getenv("CHRONO_ROOT");
    std::string root = env_root ? std::string(env_root) : std::string(CHRONO_SHOWCASE_ROOT);
    if (!root.empty() && root.back() != '/') root += '/';
    SetChronoDataPath(root + "data/");
    vehicle::SetVehicleDataPath(root + "data/vehicle/");

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    // flat terrain patch (portable)
    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo; minfo.mu = 0.9f; minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

    // Audi, at origin, facing +x
    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.55), QUNIT));
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetWheelVisualizationType(VisualizationType::MESH);
    {
        auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
        audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : audi.GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_TMeasyTire.json"));
                tire->SetStepsize(1e-3);
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddDirectionalLight(ChVector3f(-0.45f, -0.25f, -0.85f), ChColor(1.4f, 1.37f, 1.3f));
    manager->scene->SetAmbientLight(ChColor(0.35f, 0.35f, 0.37f));
    manager->scene->SetEnvMap(GetChronoDataFile("sensor/textures/sky_2_4k.hdr"));

    // Depth camera attached to the chassis; pose is updated each frame to orbit the car.
    // ChDepthCamera(parent, updateRate, offsetPose, w, h, hFOV, maxDepth=1000, lens=PINHOLE) -- no supersample arg.
    auto cam = chrono_types::make_shared<ChDepthCamera>(
        audi.GetChassisBody(), 500.0f, ChFrame<double>(), 1280, 720, (float)(CH_PI / 3), 20.0f /*maxDepth*/);
    cam->SetName("showcase_camera_depth");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("demos_live/showcase_out/camera_depth/"));
    manager->AddSensor(cam);

    printf("Showcase Depth camera (Metal). PNGs -> demos_live/showcase_out/camera_depth/\n");
    const double step = 2e-3;
    const int n_frames = 150;
    const ChVector3d center(0, 0, 0.8);
    double time = 0;
    DriverInputs in; in.m_throttle = 0; in.m_steering = 0; in.m_braking = 1.0;  // parked
    for (int frame = 0; frame < n_frames; ++frame) {
        double ang = 2.0 * CH_PI * frame / (double)n_frames;
        ChVector3d pos(6.0 * std::cos(ang), 6.0 * std::sin(ang), 1.8);
        ChVector3d d = (center - pos).GetNormalized();
        cam->SetOffsetPose(ChFrame<double>(pos, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z()))));

        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();
        time += step;
    }
    return 0;
}
