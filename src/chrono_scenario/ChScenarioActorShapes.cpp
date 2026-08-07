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

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/core/ChTypes.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_scenario/ChScenarioActorShapes.h"

namespace chrono {
namespace scenario {

namespace {

/// Bounding box of Chrono's sedan chassis mesh, measured from the .obj. Scaling an actor's
/// reported dimensions against these lets one mesh stand in for a range of vehicle sizes.
/// The mesh sits on the ground plane (z from ~0 up) and is centered longitudinally.
constexpr double kSedanMeshLength = 4.89;
constexpr double kSedanMeshWidth = 1.85;
constexpr double kSedanMeshHeight = 1.47;

/// Radius of Chrono's sedan tire mesh, likewise measured from the .obj.
constexpr double kSedanTireRadius = 0.34;

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

/// Attach a scaled chassis mesh plus four wheels.
void AddVehicle(std::shared_ptr<ChBody> body, const ChScenarioActor& actor, const ChActorVisualStyle& style) {
    double l = actor.length > 0 ? actor.length : 4.5;
    double w = actor.width > 0 ? actor.width : 1.8;
    double h = actor.height > 0 ? actor.height : 1.5;

    std::string chassis_mesh = style.car_chassis_mesh.empty()
                                   ? vehicle::GetVehicleDataFile("sedan/sedan_chassis_vis.obj")
                                   : style.car_chassis_mesh;

    ChVector3d scale(l / kSedanMeshLength, w / kSedanMeshWidth, h / kSedanMeshHeight);

    // The mesh is centered longitudinally and sits on the ground, so it is placed at the box
    // center in x and y but at ground level in z -- not at the box center height.
    ChVector3d chassis_pos(actor.center_offset.x(), actor.center_offset.y(), 0);

    auto chassis = chrono_types::make_shared<ChVisualShapeModelFile>();
    chassis->SetFilename(chassis_mesh);
    chassis->SetScale(scale);
    chassis->SetColor(style.vehicle_color);
    body->AddVisualShape(chassis, ChFrame<double>(chassis_pos, QUNIT));

    if (!style.draw_wheels)
        return;

    std::string wheel_mesh = style.car_wheel_mesh.empty()
                                 ? vehicle::GetVehicleDataFile("sedan/sedan_tire.obj")
                                 : style.car_wheel_mesh;

    // The scenario reports a bounding box but no axle positions -- esmini does not expose the
    // catalog's FrontAxle/RearAxle entries -- so the wheelbase and track are proportions of the
    // box. They are close enough that the vehicle reads as wheeled, which is the point.
    double wheelbase = 0.58 * l;
    double half_track = 0.5 * (w - 0.22 * w);
    double radius = std::min(kSedanTireRadius * (h / kSedanMeshHeight), 0.45 * h);
    double wheel_scale = radius / kSedanTireRadius;

    // Wheel longitudinal positions straddle the box center; the reference point is the rear axle,
    // so the rear pair lands near it and the front pair a wheelbase ahead.
    double x_rear = actor.center_offset.x() - 0.5 * wheelbase;
    double x_front = actor.center_offset.x() + 0.5 * wheelbase;

    for (double x : {x_rear, x_front}) {
        for (double side : {-1.0, 1.0}) {
            auto wheel = chrono_types::make_shared<ChVisualShapeModelFile>();
            wheel->SetFilename(wheel_mesh);
            wheel->SetScale(ChVector3d(wheel_scale, wheel_scale, wheel_scale));
            wheel->SetColor(ChColor(0.06f, 0.06f, 0.07f));
            body->AddVisualShape(
                wheel, ChFrame<double>(ChVector3d(x, side * half_track + actor.center_offset.y(), radius),
                                       QUNIT));
        }
    }
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
            AddVehicle(body, actor, style);
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
