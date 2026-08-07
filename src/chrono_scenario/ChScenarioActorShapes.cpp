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
// Visual bodies for OpenSCENARIO actors.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_scenario/ChScenarioActorShapes.h"

namespace chrono {
namespace scenario {

namespace {

std::shared_ptr<ChVisualMaterial> MakeMaterial(const ChColor& color) {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(color);
    return mat;
}

/// Attach a bounding box sized to the actor, centered on its reported box center.
void AddBoundingBox(std::shared_ptr<ChBody> body, const ChScenarioActor& actor, const ChColor& color) {
    double l = actor.length > 0 ? actor.length : 4.5;
    double w = actor.width > 0 ? actor.width : 1.8;
    double h = actor.height > 0 ? actor.height : 1.5;

    auto box = chrono_types::make_shared<ChVisualShapeBox>(l, w, h);
    box->SetMaterial(0, MakeMaterial(color));
    body->AddVisualShape(box, ChFrame<double>(actor.center_offset, QUNIT));
}

/// A complete Chrono vehicle's visual geometry, flattened into one rigid set of shapes.
///
/// Every shape is recorded with its pose relative to the vehicle's chassis reference frame, so
/// the whole assembly can be replayed onto a single body. Shapes are shared, not copied.
struct VehicleVisualTemplate {
    std::vector<ChVisualShapeInstance> shapes;

    /// Height of the chassis reference frame above the ground, derived from the vehicle itself:
    /// with the wheel centres a tire radius off the ground, the frame sits that far above them
    /// less the spindle's own offset. Needed because OpenSCENARIO places an actor at ground
    /// level, so the assembly has to be lifted or the tires sink into the road.
    double ref_height = 0;

    double wheelbase = 0;
    bool valid = false;
};

/// Build a Chrono vehicle in a throwaway system and harvest everything it draws.
///
/// This is the point of the exercise: Chrono::Vehicle already knows how a model goes together,
/// so the geometry is taken from the assembled vehicle rather than reconstructed from measured
/// constants. The scratch system is discarded; the visual shapes are shared_ptr and outlive it.
VehicleVisualTemplate BuildVehicleTemplate(const std::string& vehicle_json, const std::string& tire_json) {
    VehicleVisualTemplate tpl;

    try {
        ChSystemNSC scratch;
        vehicle::WheeledVehicle veh(&scratch, vehicle_json);
        veh.Initialize(ChCoordsys<>(VNULL, QUNIT));
        // Body and wheels only. Suspension and steering are internal parts; drawn as primitives
        // they poke through the bodywork, and a traffic actor gains nothing from them.
        veh.SetChassisVisualizationType(VisualizationType::MESH);
        veh.SetSuspensionVisualizationType(VisualizationType::NONE);
        veh.SetSteeringVisualizationType(VisualizationType::NONE);
        veh.SetWheelVisualizationType(VisualizationType::MESH);

        // Tires are initialized purely for their visual meshes; no powertrain is needed, since
        // this vehicle is never stepped.
        for (auto& axle : veh.GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = vehicle::ReadTireJSON(tire_json);
                veh.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }

        // The chassis reference frame is the origin the assembly is expressed against, and is
        // also what OpenSCENARIO's reference point has to be reconciled with later.
        ChFrame<double> chassis_frame(veh.GetPos(), veh.GetRot());
        ChFrame<double> to_chassis = chassis_frame.GetInverse();

        // Ride height, from the spindles and the tire radius rather than a measured constant.
        int num_axles = static_cast<int>(veh.GetAxles().size());
        double spindle_z = 0;
        for (int a = 0; a < num_axles; a++) {
            spindle_z += 0.5 * (veh.GetSpindlePos(a, vehicle::VehicleSide::LEFT).z() +
                                veh.GetSpindlePos(a, vehicle::VehicleSide::RIGHT).z());
        }
        spindle_z /= std::max(num_axles, 1);

        double tire_radius = 0;
        if (num_axles > 0) {
            auto tire = veh.GetAxle(0)->GetWheel(vehicle::VehicleSide::LEFT)->GetTire();
            if (tire)
                tire_radius = tire->GetRadius();
        }
        tpl.ref_height = tire_radius - (spindle_z - chassis_frame.GetPos().z());
        tpl.wheelbase = veh.GetWheelbase();

        for (const auto& body : scratch.GetBodies()) {
            const auto& model = body->GetVisualModel();
            if (!model)
                continue;

            ChFrame<double> body_frame = to_chassis * body->GetVisualModelFrame();
            for (const auto& si : model->GetShapeInstances()) {
                ChVisualShapeInstance out = si;
                out.frame = body_frame * si.frame;
                tpl.shapes.push_back(out);
            }
        }

        if (tpl.shapes.empty())
            return tpl;

        tpl.valid = true;
    } catch (const std::exception& e) {
        std::cerr << "ChScenarioActorShapes: could not build vehicle model " << vehicle_json << " ("
                  << e.what() << "); actors will fall back to bounding boxes" << std::endl;
    }

    return tpl;
}

/// Templates are cached: building a vehicle is far too expensive to repeat per actor, and a
/// scenario may well contain several of the same category.
const VehicleVisualTemplate& CachedVehicleTemplate(const std::string& vehicle_json,
                                                   const std::string& tire_json) {
    static std::map<std::string, VehicleVisualTemplate> cache;
    auto it = cache.find(vehicle_json);
    if (it == cache.end())
        it = cache.emplace(vehicle_json, BuildVehicleTemplate(vehicle_json, tire_json)).first;

    return it->second;
}

/// Attach a two-wheeler built from primitives.
///
/// Chrono ships no bicycle or motorbike model, and the category mapping would otherwise hand a
/// 1.8 m bicycle a 4.9 m sedan -- worse than a box, because it is confidently the wrong thing.
/// Two wheels, a frame and a rider at least give the right silhouette and footprint, which is
/// what matters for a vulnerable-road-user scenario.
void AddTwoWheeler(std::shared_ptr<ChBody> body, const ChScenarioActor& actor, const ChColor& color) {
    double l = actor.length > 0 ? actor.length : 1.8;
    double w = actor.width > 0 ? actor.width : 0.5;
    double h = actor.height > 0 ? actor.height : 1.7;

    auto frame_mat = MakeMaterial(color);
    auto tire_mat = MakeMaterial(ChColor(0.05f, 0.05f, 0.06f));
    auto rider_mat = MakeMaterial(ChColor(0.20f, 0.20f, 0.24f));

    double wheel_r = std::min(0.35 * h, 0.5 * l - 0.05);
    double x = actor.center_offset.x();
    double y = actor.center_offset.y();

    // Cylinder axes run along local Z, so a wheel is laid over about X to spin around Y.
    ChQuaterniond wheel_rot = QuatFromAngleX(CH_PI_2);
    for (double side : {-1.0, 1.0}) {
        auto wheel = chrono_types::make_shared<ChVisualShapeCylinder>(wheel_r, 0.06);
        wheel->SetMaterial(0, tire_mat);
        body->AddVisualShape(wheel,
                             ChFrame<double>(ChVector3d(x + side * (0.5 * l - wheel_r), y, wheel_r),
                                             wheel_rot));
    }

    auto frame = chrono_types::make_shared<ChVisualShapeBox>(0.75 * l, 0.4 * w, 0.22 * h);
    frame->SetMaterial(0, frame_mat);
    body->AddVisualShape(frame, ChFrame<double>(ChVector3d(x, y, wheel_r + 0.10 * h), QUNIT));

    // A rider, since these categories are almost always ridden in a scenario.
    double torso_len = 0.32 * h;
    auto torso = chrono_types::make_shared<ChVisualShapeCapsule>(0.16 * w + 0.06, torso_len);
    torso->SetMaterial(0, rider_mat);
    body->AddVisualShape(
        torso, ChFrame<double>(ChVector3d(x - 0.05 * l, y, h - 0.5 * torso_len - 0.11 * h),
                               QuatFromAngleX(CH_PI_2)));

    auto head = chrono_types::make_shared<ChVisualShapeSphere>(0.075 * h);
    head->SetMaterial(0, rider_mat);
    body->AddVisualShape(head, ChFrame<double>(ChVector3d(x - 0.05 * l, y, h - 0.075 * h), QUNIT));
}

/// Attach a whole Chrono vehicle's geometry, chosen by the actor's category.
///
/// Placement reconciles two origins. Chrono expresses the assembly against the chassis reference
/// frame; OpenSCENARIO places an actor by its rear axle centre at ground level. The template is
/// therefore shifted so its rear axle lands on the actor origin -- approximated here by the
/// bounding box centre offset the scenario reports, which is measured from that same point.
bool AddVehicle(std::shared_ptr<ChBody> body, const ChScenarioActor& actor, const ChActorVisualStyle& style) {
    if (actor.object_category == ChScenarioVehicleCategory::BICYCLE ||
        actor.object_category == ChScenarioVehicleCategory::MOTORBIKE) {
        AddTwoWheeler(body, actor, ChColor(0.90f, 0.55f, 0.10f));
        return true;
    }

    bool large = actor.object_category == ChScenarioVehicleCategory::BUS ||
                 actor.object_category == ChScenarioVehicleCategory::TRUCK ||
                 actor.object_category == ChScenarioVehicleCategory::SEMITRAILER ||
                 actor.length > 8.0;

    std::string vehicle_json = large
        ? (style.bus_vehicle_json.empty() ? vehicle::GetVehicleDataFile("citybus/vehicle/CityBus_Vehicle.json")
                                          : style.bus_vehicle_json)
        : (style.car_vehicle_json.empty() ? vehicle::GetVehicleDataFile("sedan/vehicle/Sedan_Vehicle.json")
                                          : style.car_vehicle_json);
    std::string tire_json = large
        ? (style.bus_tire_json.empty() ? vehicle::GetVehicleDataFile("citybus/tire/CityBus_TMeasyTire.json")
                                       : style.bus_tire_json)
        : (style.car_tire_json.empty() ? vehicle::GetVehicleDataFile("sedan/tire/Sedan_TMeasyTire.json")
                                       : style.car_tire_json);

    const VehicleVisualTemplate& tpl = CachedVehicleTemplate(vehicle_json, tire_json);
    if (!tpl.valid)
        return false;

    // Two origins to reconcile. Chrono expresses the assembly against the chassis reference
    // frame; OpenSCENARIO places the actor by its rear axle centre at ground level. Lifting by
    // the ride height puts the tires on the road, and the longitudinal shift lines the model's
    // wheelbase up with where the scenario says the axles are.
    ChVector3d shift(actor.center_offset.x() - 0.5 * tpl.wheelbase, actor.center_offset.y(),
                     tpl.ref_height);
    for (const auto& si : tpl.shapes)
        body->AddVisualShape(si.shape, ChFrame<double>(shift + si.frame.GetPos(), si.frame.GetRot()));

    return true;
}

/// Attach a jointed figure approximating a person.
///
/// Chrono ships no pedestrian mesh, and esmini's walkman.osgb is an OpenSceneGraph binary this
/// build cannot read. Primitives at least give a perception model a upright, limbed silhouette
/// with a head rather than a rectangular slab.
void AddPedestrian(std::shared_ptr<ChBody> body, const ChScenarioActor& actor, const ChColor& color) {
    double h = actor.height > 0 ? actor.height : 1.8;
    double w = actor.width > 0 ? actor.width : 0.5;

    auto mat = MakeMaterial(color);
    auto skin = MakeMaterial(ChColor(0.80f, 0.66f, 0.55f));

    // Proportions of a standing adult, as fractions of total height.
    double head_r = 0.065 * h;
    double torso_len = 0.30 * h;
    double torso_r = 0.5 * std::min(w, 0.30 * h);
    double leg_len = 0.42 * h;
    double leg_r = 0.055 * h;
    double arm_len = 0.30 * h;
    double arm_r = 0.040 * h;

    double x = actor.center_offset.x();
    double y = actor.center_offset.y();

    // Capsule axes run along local Y, so each limb is rotated upright about X.
    ChQuaterniond upright = QuatFromAngleX(CH_PI_2);

    auto torso = chrono_types::make_shared<ChVisualShapeCapsule>(torso_r, torso_len);
    torso->SetMaterial(0, mat);
    body->AddVisualShape(torso, ChFrame<double>(ChVector3d(x, y, leg_len + 0.5 * torso_len), upright));

    auto head = chrono_types::make_shared<ChVisualShapeSphere>(head_r);
    head->SetMaterial(0, skin);
    body->AddVisualShape(head, ChFrame<double>(ChVector3d(x, y, h - head_r), QUNIT));

    for (double side : {-1.0, 1.0}) {
        auto leg = chrono_types::make_shared<ChVisualShapeCapsule>(leg_r, leg_len);
        leg->SetMaterial(0, mat);
        body->AddVisualShape(
            leg, ChFrame<double>(ChVector3d(x, y + side * 0.9 * leg_r, 0.5 * leg_len), upright));

        auto arm = chrono_types::make_shared<ChVisualShapeCapsule>(arm_r, arm_len);
        arm->SetMaterial(0, mat);
        body->AddVisualShape(
            arm, ChFrame<double>(ChVector3d(x, y + side * (torso_r + arm_r),
                                            leg_len + torso_len - 0.5 * arm_len),
                                 upright));
    }
}

}  // namespace

std::shared_ptr<ChBody> CreateScenarioActorBody(ChSystem& sys,
                                                const ChScenarioActor& actor,
                                                const ChActorVisualStyle& style) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName(actor.name.empty() ? "scenario_actor" : actor.name.c_str());
    body->EnableCollision(false);

    // Fixed, and still repositioned every step by SetPos/SetRot. Verified that the renderers
    // track a fixed body's transform, so there is no reason to leave these in the solver: doing
    // so only adds an active body per actor.
    //
    // What does matter is *when* the body is created. Both renderers bind the bodies present when
    // their scene is built, so an actor created after that renders as nothing at all. Callers
    // must build these before constructing a visual system or sensor manager.
    body->SetFixed(true);

    if (style.boxes_only) {
        AddBoundingBox(body, actor, style.misc_color);
        sys.Add(body);
        return body;
    }

    switch (actor.object_type) {
        case ChScenarioObjectType::VEHICLE:
            if (!AddVehicle(body, actor, style))
                AddBoundingBox(body, actor, style.misc_color);
            break;
        case ChScenarioObjectType::PEDESTRIAN:
            AddPedestrian(body, actor, style.pedestrian_color);
            break;
        default:
            // Misc objects are barriers, poles, patches and the like -- a box is genuinely the
            // right shape for most of them, and the scenario gives nothing better to go on.
            AddBoundingBox(body, actor, style.misc_color);
            break;
    }

    sys.Add(body);
    return body;
}

}  // end namespace scenario
}  // end namespace chrono
