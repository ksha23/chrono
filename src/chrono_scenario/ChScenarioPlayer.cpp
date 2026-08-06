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
// Driving scenario loaded from an ASAM OpenSCENARIO (.xosc) file.
//
// =============================================================================

#include <algorithm>
#include <iostream>

#include "chrono_scenario/ChScenarioPlayer.h"

#include "chrono_vehicle/ChWorldFrame.h"

#include "esminiLib.hpp"

namespace chrono {
namespace scenario {

using vehicle::ChWorldFrame;

namespace {
constexpr int kSeOk = 0;

/// SE_Init use_viewer bitmask: 0 instantiates no viewer at all. Chrono::Sensor does the rendering,
/// and esmini's viewer is not even built when esmini is configured with USE_OSG=OFF.
constexpr int kNoViewer = 0;

/// SE_Init threads: 0 runs the scenario engine on the calling thread, which keeps stepping
/// deterministic with respect to the Chrono step loop.
constexpr int kSingleThread = 0;

/// Convert a Chrono world pose to the position and ISO Euler angles esmini expects.
void PoseToEsmini(const ChCoordsys<>& pose, double& x, double& y, double& z, double& h, double& p, double& r) {
    ChVector3d pos_ISO = ChWorldFrame::ToISO(pose.pos);
    x = pos_ISO.x();
    y = pos_ISO.y();
    z = pos_ISO.z();

    // ChQuaternion::GetCardanAnglesZYX returns the Z-Y'-X'' intrinsic angles packed as
    // (.x() = pitch, .y() = roll, .z() = yaw) -- the component order does not follow the
    // "heading, bank, attitude" naming in its doc comment. Verified numerically.
    ChVector3d angles = pose.rot.GetCardanAnglesZYX();
    p = angles.x();
    r = angles.y();
    h = angles.z();
}

/// Convert an esmini object state to a Chrono actor record.
ChScenarioActor StateToActor(const SE_ScenarioObjectState& state) {
    ChScenarioActor actor;

    actor.id = state.id;
    if (const char* name = SE_GetObjectName(state.id))
        actor.name = name;

    ChVector3d pos = ChWorldFrame::FromISO(ChVector3d(state.x, state.y, state.z));
    ChQuaterniond rot = QuatFromAngleZ(state.h) * QuatFromAngleY(state.p) * QuatFromAngleX(state.r);
    actor.pose = ChCoordsys<>(pos, rot);

    actor.speed = state.speed;
    actor.length = state.length;
    actor.width = state.width;
    actor.height = state.height;
    actor.center_offset = ChVector3d(state.centerOffsetX, state.centerOffsetY, state.centerOffsetZ);
    actor.object_type = state.objectType;
    actor.object_category = state.objectCategory;
    actor.road_id = state.roadId;
    actor.lane_id = state.laneId;
    actor.s = state.s;
    actor.lane_offset = state.laneOffset;

    return actor;
}
}  // namespace

// -----------------------------------------------------------------------------------------------
// Reference point conversion
// -----------------------------------------------------------------------------------------------

namespace {
/// World position of the center of a wheeled vehicle's rearmost axle.
ChVector3d RearAxleCenter(const vehicle::ChWheeledVehicle& vehicle) {
    int rear = static_cast<int>(vehicle.GetAxles().size()) - 1;
    return 0.5 * (vehicle.GetSpindlePos(rear, vehicle::VehicleSide::LEFT) +
                  vehicle.GetSpindlePos(rear, vehicle::VehicleSide::RIGHT));
}
}  // namespace

ChVector3d GetScenarioRefPointOffset(const vehicle::ChWheeledVehicle& vehicle) {
    // Express the rear axle center in the chassis reference frame. Note this uses the chassis
    // *reference frame* (ChVehicle::GetPos/GetRot), not the chassis body's center of mass.
    ChCoordsys<> chassis(vehicle.GetPos(), vehicle.GetRot());
    return chassis.TransformPointParentToLocal(RearAxleCenter(vehicle));
}

ChCoordsys<> GetScenarioRefPose(const vehicle::ChWheeledVehicle& vehicle) {
    return ChCoordsys<>(RearAxleCenter(vehicle), vehicle.GetRot());
}

bool ChScenarioPlayer::m_instance_live = false;

ChScenarioPlayer::ChScenarioPlayer()
    : m_initialized(false), m_disable_ctrls(false), m_record(false), m_ego_id(-1) {}

ChScenarioPlayer::~ChScenarioPlayer() {
    if (!m_initialized)
        return;

    SE_Close();
    m_instance_live = false;
}

bool ChScenarioPlayer::Initialize(const std::string& xosc_file, int ego_index) {
    if (m_initialized) {
        std::cerr << "ChScenarioPlayer::Initialize(): scenario already loaded from " << m_filename
                  << std::endl;
        return false;
    }

    // esminiLib holds the scenario in process-global state, so a second live instance would
    // silently unload the first one's scenario out from under it.
    if (m_instance_live) {
        std::cerr << "ChScenarioPlayer::Initialize(): another ChScenarioPlayer is already "
                     "initialized; esminiLib supports only one loaded scenario per process"
                  << std::endl;
        return false;
    }

    if (SE_Init(xosc_file.c_str(), m_disable_ctrls ? 1 : 0, kNoViewer, kSingleThread, m_record ? 1 : 0) !=
        kSeOk) {
        std::cerr << "ChScenarioPlayer::Initialize(): failed to load OpenSCENARIO file " << xosc_file
                  << std::endl;
        return false;
    }

    int num_objects = SE_GetNumberOfObjects();
    if (num_objects <= 0) {
        std::cerr << "ChScenarioPlayer::Initialize(): scenario contains no entities" << std::endl;
        SE_Close();
        return false;
    }

    if (ego_index < 0 || ego_index >= num_objects) {
        std::cerr << "ChScenarioPlayer::Initialize(): ego index " << ego_index << " out of range ("
                  << num_objects << " entities)" << std::endl;
        SE_Close();
        return false;
    }

    m_ego_id = SE_GetId(ego_index);
    if (m_ego_id < 0) {
        std::cerr << "ChScenarioPlayer::Initialize(): could not resolve the ego object ID" << std::endl;
        SE_Close();
        return false;
    }

    m_filename = xosc_file;
    m_initialized = true;
    m_instance_live = true;
    return true;
}

std::string ChScenarioPlayer::GetOdrFilename() const {
    if (!m_initialized)
        return "";

    const char* name = SE_GetODRFilename();
    return name ? std::string(name) : std::string();
}

// -----------------------------------------------------------------------------------------------
// Co-simulation
// -----------------------------------------------------------------------------------------------

bool ChScenarioPlayer::ReportEgoState(const ChCoordsys<>& pose, double speed) {
    if (!m_initialized)
        return false;

    double x, y, z, h, p, r;
    PoseToEsmini(pose, x, y, z, h, p, r);

    if (SE_ReportObjectPos(m_ego_id, x, y, z, h, p, r) != kSeOk)
        return false;

    // Reported separately: the scenario's own triggers and controllers reason about speed, and
    // esmini does not differentiate it from the reported positions.
    return SE_ReportObjectSpeed(m_ego_id, speed) == kSeOk;
}

bool ChScenarioPlayer::ReportEgoState(const vehicle::ChWheeledVehicle& vehicle) {
    return ReportEgoState(GetScenarioRefPose(vehicle), vehicle.GetSpeed());
}

void ChScenarioPlayer::Advance(double step) {
    if (!m_initialized)
        return;

    SE_StepDT(step);
}

double ChScenarioPlayer::GetTime() const {
    if (!m_initialized)
        return 0;

    return SE_GetSimulationTime();
}

bool ChScenarioPlayer::IsComplete() const {
    if (!m_initialized)
        return true;

    return SE_GetQuitFlag() == 1;
}

// -----------------------------------------------------------------------------------------------
// Actors
// -----------------------------------------------------------------------------------------------

int ChScenarioPlayer::GetNumObjects() const {
    if (!m_initialized)
        return 0;

    int n = SE_GetNumberOfObjects();
    return n < 0 ? 0 : n;
}

std::vector<ChScenarioActor> ChScenarioPlayer::GetAllObjects() const {
    std::vector<ChScenarioActor> actors;

    int n = GetNumObjects();
    actors.reserve(n);

    for (int i = 0; i < n; i++) {
        int id = SE_GetId(i);
        if (id < 0)
            continue;

        SE_ScenarioObjectState state;
        if (SE_GetObjectState(id, &state) != kSeOk)
            continue;

        actors.push_back(StateToActor(state));
    }

    return actors;
}

std::vector<ChScenarioActor> ChScenarioPlayer::GetActors() const {
    std::vector<ChScenarioActor> actors = GetAllObjects();

    actors.erase(std::remove_if(actors.begin(), actors.end(),
                                [this](const ChScenarioActor& a) { return a.id == m_ego_id; }),
                 actors.end());

    return actors;
}

ChScenarioActor ChScenarioPlayer::GetObject(int object_id) const {
    ChScenarioActor actor;
    if (!m_initialized)
        return actor;

    SE_ScenarioObjectState state;
    if (SE_GetObjectState(object_id, &state) != kSeOk)
        return actor;

    return StateToActor(state);
}

}  // end namespace scenario
}  // end namespace chrono
