// SHOWCASE (METAL backend): shallow DEPTH OF FIELD (physical-camera bokeh).
// A stationary Audi on flat terrain under the shipped sky_2_4k HDR. The camera slowly orbits with a large
// aperture and the focal plane set on the car, so the near/far terrain and horizon melt into bokeh while the
// Audi stays sharp. HEADLESS: writes 150 PNGs to demos_live/showcase_out/physcam_dof/ then returns.
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

    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo; minfo.mu = 0.9f; minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

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
    // Depth of field: aperture sized so the WHOLE ~4.8 m-long car sits inside the in-focus zone (front
    // bumper to tail sharp) while the distant terrain/trees still melt into bokeh. A larger aperture (0.35)
    // made the depth of field so shallow that only the car's mid-slice was sharp.
    manager->scene->SetDepthOfField(0.055f /*aperture radius*/, 6.0f /*focal distance = orbit radius*/);

    const double radius = 6.0, height = 1.8;
    const ChVector3d look(0, 0, 0.8);
    auto orbitPose = [&](double ang) {
        ChVector3d off(radius * std::cos(ang), radius * std::sin(ang), height);
        ChVector3d d = (look - off).GetNormalized();
        return ChFrame<double>(off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));
    };

    // DoF needs many samples per pixel for smooth bokeh -> ss=4.
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        audi.GetChassisBody(), 500.0f, orbitPose(0.0), 1280, 720,
        (float)(CH_PI / 3), 4 /*ss: DoF needs samples*/, CameraLensModelType::PINHOLE, false /*GI*/, true /*denoiser*/);
    cam->SetName("physcam_dof");
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("demos_live/showcase_out/physcam_dof/"));
    manager->AddSensor(cam);

    printf("Depth-of-field showcase (Metal). PNGs -> demos_live/showcase_out/physcam_dof/\n");
    const double step = 2e-3;
    const int nframes = 150;
    double time = 0;
    DriverInputs in; in.m_throttle = 0; in.m_steering = 0; in.m_braking = 1.0;  // parked
    for (int f = 0; f < nframes; ++f) {
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        // Slow orbit (quarter turn over 150 frames) so the bokeh has time to read.
        cam->SetOffsetPose(orbitPose(CH_2PI * f / nframes));  // full orbit (synced with the other demos)
        manager->Update();
        time += step;
    }
    return 0;
}
