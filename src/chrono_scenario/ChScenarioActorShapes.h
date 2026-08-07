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
// A scenario reports each actor's pose, type and bounding box, but no geometry:
// its model3d reference points at an esmini .osgb file, which Chrono cannot
// read. Something therefore has to stand in for the actor in the Chrono scene.
//
// A bounding box is the obvious stand-in and the wrong one. The bring-up behind
// this work measured openpilot against an untextured, wheel-less chassis and got
// pure noise out of it -- lead probability wandering 0.00 to 0.42, distance
// jumping between 34 and 109 m -- against a clean 0.92-to-1.00 lock on a proper
// vehicle. Obstacle *fidelity*, not presence, is what decides detection, so an
// actor drawn as a box is not something a perception model can be fairly tested
// against. These builders substitute real meshes instead.
//
// =============================================================================

#ifndef CH_SCENARIO_ACTOR_SHAPES_H
#define CH_SCENARIO_ACTOR_SHAPES_H

#include <memory>
#include <string>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_scenario/ChApiScenario.h"
#include "chrono_scenario/ChScenarioPlayer.h"

namespace chrono {
namespace scenario {

/// @addtogroup scenario_module
/// @{

/// OpenSCENARIO object types, as reported in ChScenarioActor::object_type.
/// These mirror esmini's Object::Type.
namespace ChScenarioObjectType {
enum : int {
    NONE = 0,
    VEHICLE = 1,
    PEDESTRIAN = 2,
    MISC_OBJECT = 3
};
}  // namespace ChScenarioObjectType

/// Vehicle sub-categories, as reported in ChScenarioActor::object_category.
///
/// \warning The category integer is only meaningful once the object type is known -- esmini
/// stores it as a bare int, so the same value means something different for a pedestrian or a
/// misc object. Always branch on object_type first.
namespace ChScenarioVehicleCategory {
enum : int {
    CAR = 0,
    VAN = 1,
    TRUCK = 2,
    SEMITRAILER = 3,
    TRAILER = 4,
    BUS = 5,
    MOTORBIKE = 6,
    BICYCLE = 7,
    TRAIN = 8,
    TRAM = 9
};
}  // namespace ChScenarioVehicleCategory

/// How scenario actors are drawn.
struct ChActorVisualStyle {
    /// Wavefront mesh used for vehicle bodies. Empty selects Chrono's sedan.
    /// The mesh is scaled to each actor's reported bounding box, so one mesh serves a fleet.
    std::string car_chassis_mesh;

    /// Wavefront mesh used for one wheel. Empty selects Chrono's sedan tire.
    std::string car_wheel_mesh;

    /// Draw wheels on vehicles (default: true).
    /// Worth leaving on: a wheel-less chassis is precisely the case that read as noise.
    bool draw_wheels = true;

    /// Fall back to plain bounding boxes for everything. Cheap, and useful when the point of a
    /// run is throughput rather than perception.
    bool boxes_only = false;

    ChColor vehicle_color = ChColor(0.75f, 0.12f, 0.10f);
    ChColor pedestrian_color = ChColor(0.15f, 0.25f, 0.65f);
    ChColor misc_color = ChColor(0.55f, 0.55f, 0.58f);
};

/// Create a visual body standing in for a scenario actor, and add it to the system.
///
/// The body is fixed and carries no collision geometry: esmini drives these kinematically, so
/// they are repositioned each step with SetPos/SetRot rather than simulated. Geometry is placed
/// relative to the actor's OpenSCENARIO reference point (the rear axle center for a vehicle),
/// using the bounding box center offset the scenario reports.
///
/// Vehicles get a scaled chassis mesh plus four wheels; pedestrians a jointed figure built from
/// primitives, since no pedestrian mesh ships with Chrono; anything else a bounding box.
ChApiScenario std::shared_ptr<ChBody> CreateScenarioActorBody(ChSystem& sys,
                                                              const ChScenarioActor& actor,
                                                              const ChActorVisualStyle& style = {});

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
