// SHOWCASE (Metal RT): DISTANCE FOG.
// A gray-blue scattering fog is applied so the flat terrain and the far side of the Audi fade into the haze
// with distance. Headless: full orbit, 150 PNG frames saved to demos_live/showcase_out/fog/. No live window.
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
#include "chrono_sensor/sensors/ChCameraSensor.h"
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

    // flat terrain patch (portable) -- the long tiles trail off into the fog
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
    manager->scene->AddDirectionalLight(ChVector3f(-0.45f, -0.25f, -0.85f), ChColor(1.2f, 1.2f, 1.25f));
    manager->scene->SetAmbientLight(ChColor(0.45f, 0.47f, 0.52f));
    // gray-blue background that the fog blends toward, and moderate exponential fog scattering. The
    // background gradient IS the fog colour, so distant tiles dissolve seamlessly into the haze.
    manager->scene->SetBackgroundGradient(ChColor(0.70f, 0.75f, 0.82f), ChColor(0.80f, 0.83f, 0.88f));
    manager->scene->SetFog(ChColor(0.78f, 0.82f, 0.88f), 0.075f);

    ChVector3d look(0, 0, 0.8);
    ChVector3d cam_off(6.0, 0.0, 1.8);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(), 500.0f, cam_pose, 1280, 720,
                   (float)(CH_PI / 3), 2, CameraLensModelType::PINHOLE, false /*GI*/, false /*denoiser*/);
    cam->SetUseFog(true);  // <-- REQUIRED: without this the shader leaves fogScatter=0 and no fog is applied
    cam->SetName("showcase_fog");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("demos_live/showcase_out/fog/"));
    manager->AddSensor(cam);

    printf("Showcase fog (Metal). PNGs -> demos_live/showcase_out/fog/. 150 frames...\n");
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
    DriverInputs in; in.m_throttle = 0; in.m_steering = 0; in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        double ang = 2.0 * CH_PI * (double)f / (double)nframes;  // full orbit
        cam_off = ChVector3d(6.0 * std::cos(ang), 6.0 * std::sin(ang), 1.8);
        d = (look - cam_off).GetNormalized();
        cam->SetOffsetPose(ChFrame<double>(cam_off,
            QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z()))));

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
