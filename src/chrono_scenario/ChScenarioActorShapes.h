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
///
/// Vehicles are drawn by importing a whole Chrono vehicle rather than by assembling meshes.
/// Chrono::Vehicle already knows how a given model goes together -- chassis, suspension,
/// steering, wheels, rims and tires, each at its own place -- and reproducing that by hand means
/// duplicating geometry that is already written down, which is how a hand-built version ends up
/// with tires but no rims and a ride height off by the chassis reference offset.
struct ChActorVisualStyle {
    /// Vehicle JSON used for cars and vans. Empty selects Chrono's sedan.
    std::string car_vehicle_json;
    std::string car_tire_json;

    /// Vehicle JSON used for buses, trucks and other large vehicles. Empty selects the city bus.
    std::string bus_vehicle_json;
    std::string bus_tire_json;

    /// Fall back to plain bounding boxes for everything. Cheap, and useful when the point of a
    /// run is throughput rather than perception.
    bool boxes_only = false;

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
/// Vehicles get the complete geometry of a Chrono vehicle -- built once in a throwaway system,
/// harvested, and replayed onto this single body. Pedestrians get a jointed figure built from
/// primitives, since no pedestrian mesh ships with Chrono; anything else gets a bounding box.
///
/// The imported vehicle is picked by the actor's category, not scaled to its bounding box, so it
/// will not match a scenario's declared dimensions exactly. A real vehicle at approximately the
/// right size reads far better to a perception model than a correctly-sized distortion of one.
ChApiScenario std::shared_ptr<ChBody> CreateScenarioActorBody(ChSystem& sys,
                                                              const ChScenarioActor& actor,
                                                              const ChActorVisualStyle& style = {});

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
