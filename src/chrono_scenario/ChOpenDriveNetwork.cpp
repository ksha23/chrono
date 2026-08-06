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
// Road network loaded from an ASAM OpenDRIVE (.xodr) file.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#include "chrono_scenario/ChOpenDriveNetwork.h"

#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_thirdparty/rapidxml/rapidxml_utils.hpp"

#include "esminiRMLib.hpp"

namespace chrono {
namespace scenario {

using vehicle::ChWorldFrame;

// esmini return conventions, from esminiRMLib.hpp:
//   RM_Init, RM_GetLaneWidth*        : 0 on success, -1 on failure
//   RM_CreatePosition                : handle >= 0, -1 on failure
//   RM_Set*Position, RM_Get*Info     : >= 0 on success, < 0 on failure
namespace {
constexpr int kRmOk = 0;

/// esmini roadmanager::Position::LookAheadMode. Measuring at the lane center keeps lookahead
/// queries on the lane a controller is trying to hold rather than on the vehicle's current
/// lateral offset, which would feed its own tracking error back into the reference.
constexpr int kLookAheadAtLaneCenter = 0;

bool RmFailed(int ret) {
    return ret < 0;
}

/// Read an XML attribute, or return a default if it is absent.
const char* Attr(rapidxml::xml_node<>* node, const char* name, const char* fallback = "") {
    auto* a = node->first_attribute(name);
    return a ? a->value() : fallback;
}

double AttrDouble(rapidxml::xml_node<>* node, const char* name, double fallback) {
    auto* a = node->first_attribute(name);
    return a ? std::atof(a->value()) : fallback;
}

ChLaneMarkingStyle::Type ParseMarkingType(const std::string& s) {
    if (s == "solid")
        return ChLaneMarkingStyle::Type::SOLID;
    if (s == "broken")
        return ChLaneMarkingStyle::Type::BROKEN;
    if (s == "solid solid")
        return ChLaneMarkingStyle::Type::SOLID_SOLID;
    if (s == "solid broken")
        return ChLaneMarkingStyle::Type::SOLID_BROKEN;
    if (s == "broken solid")
        return ChLaneMarkingStyle::Type::BROKEN_SOLID;
    if (s == "none" || s.empty())
        return ChLaneMarkingStyle::Type::NONE;

    // curb, grass, botts dots, edge, custom -- present in the standard but not painted lines.
    return ChLaneMarkingStyle::Type::OTHER;
}

/// OpenDRIVE marking colors. "standard" means white.
ChColor ParseMarkingColor(const std::string& s) {
    if (s == "yellow")
        return ChColor(0.95f, 0.78f, 0.15f);
    if (s == "blue")
        return ChColor(0.20f, 0.40f, 0.90f);
    if (s == "green")
        return ChColor(0.20f, 0.70f, 0.35f);
    if (s == "red")
        return ChColor(0.85f, 0.15f, 0.15f);
    if (s == "orange")
        return ChColor(0.95f, 0.55f, 0.10f);

    return ChColor(1.0f, 1.0f, 1.0f);  // "standard" and "white"
}
}  // namespace

bool ChOpenDriveNetwork::m_instance_live = false;

ChOpenDriveNetwork::ChOpenDriveNetwork() : m_initialized(false), m_scratch(-1), m_scratch_elev(-1) {}

ChOpenDriveNetwork::~ChOpenDriveNetwork() {
    if (!m_initialized)
        return;

    if (m_scratch >= 0)
        RM_DeletePosition(m_scratch);
    if (m_scratch_elev >= 0)
        RM_DeletePosition(m_scratch_elev);
    RM_Close();

    m_instance_live = false;
}

bool ChOpenDriveNetwork::Initialize(const std::string& xodr_file) {
    if (m_initialized) {
        std::cerr << "ChOpenDriveNetwork::Initialize(): network already loaded from " << m_filename
                  << std::endl;
        return false;
    }

    // esminiRMLib holds the road network in process-global state, so a second live instance would
    // silently unload the first one's network out from under it.
    if (m_instance_live) {
        std::cerr << "ChOpenDriveNetwork::Initialize(): another ChOpenDriveNetwork is already "
                     "initialized; esminiRMLib supports only one loaded network per process"
                  << std::endl;
        return false;
    }

    if (RM_Init(xodr_file.c_str()) != kRmOk) {
        std::cerr << "ChOpenDriveNetwork::Initialize(): failed to load OpenDRIVE file " << xodr_file
                  << std::endl;
        return false;
    }

    m_scratch = RM_CreatePosition();
    m_scratch_elev = RM_CreatePosition();
    if (m_scratch < 0 || m_scratch_elev < 0) {
        std::cerr << "ChOpenDriveNetwork::Initialize(): failed to create a position handle" << std::endl;
        RM_Close();
        return false;
    }

    // Snap world positions to drivable lanes by default; callers can widen this.
    RM_SetSnapLaneTypes(m_scratch, ChLaneType::ANY_DRIVING);

    // Elevation queries want the whole paved surface, shoulders and all, not just travel lanes.
    RM_SetSnapLaneTypes(m_scratch_elev, ChLaneType::ANY_ROAD);

    ParseLaneMarkings(xodr_file);

    m_filename = xodr_file;
    m_initialized = true;
    m_instance_live = true;
    return true;
}

// -----------------------------------------------------------------------------------------------
// Lane markings
//
// esmini parses <roadMark> into its road manager but exposes none of it through either C API, so
// the .xodr is read a second time here. The XML is small next to the road geometry esmini already
// built from it, so the duplicated parse is not worth avoiding.
// -----------------------------------------------------------------------------------------------

void ChOpenDriveNetwork::ParseLaneMarkings(const std::string& xodr_file) {
    m_markings.clear();

    rapidxml::xml_document<> doc;
    std::unique_ptr<rapidxml::file<>> raw;
    try {
        raw = std::make_unique<rapidxml::file<>>(xodr_file.c_str());
        doc.parse<0>(raw->data());
    } catch (const std::exception& e) {
        std::cerr << "ChOpenDriveNetwork: could not read lane markings from " << xodr_file << " ("
                  << e.what() << "); the network will render without them" << std::endl;
        return;
    }

    auto* root = doc.first_node("OpenDRIVE");
    if (!root)
        return;

    for (auto* road = root->first_node("road"); road; road = road->next_sibling("road")) {
        unsigned int road_id = static_cast<unsigned int>(std::atoi(Attr(road, "id", "-1")));

        auto* lanes = road->first_node("lanes");
        if (!lanes)
            continue;

        // Only the first lane section is read. A road may redefine its lanes at several s
        // stations; carrying that through would require the marking geometry to be built per
        // section as well, which the generator does not do yet.
        auto* section = lanes->first_node("laneSection");
        if (!section)
            continue;

        for (const char* side : {"left", "center", "right"}) {
            auto* group = section->first_node(side);
            if (!group)
                continue;

            for (auto* lane = group->first_node("lane"); lane; lane = lane->next_sibling("lane")) {
                auto* mark = lane->first_node("roadMark");
                if (!mark)
                    continue;

                int lane_id = std::atoi(Attr(lane, "id", "0"));

                ChLaneMarkingStyle style;
                style.type = ParseMarkingType(Attr(mark, "type", "none"));
                style.width = AttrDouble(mark, "width", 0.12);
                style.height = AttrDouble(mark, "height", 0.0);
                style.color = ParseMarkingColor(Attr(mark, "color", "standard"));

                // A <type><line> child carries the dash pattern for broken marks. Solid marks
                // report length 0, which is how they are distinguished here.
                if (auto* mtype = mark->first_node("type")) {
                    if (auto* line = mtype->first_node("line")) {
                        style.dash_length = AttrDouble(line, "length", 0.0);
                        style.dash_space = AttrDouble(line, "space", 0.0);
                        style.width = AttrDouble(line, "width", style.width);
                    }
                }

                m_markings[{road_id, lane_id}] = style;
            }
        }
    }
}

ChLaneMarkingStyle ChOpenDriveNetwork::GetLaneMarkingStyle(unsigned int road_id, int lane_id) const {
    auto it = m_markings.find({road_id, lane_id});
    return it == m_markings.end() ? ChLaneMarkingStyle() : it->second;
}

std::vector<std::pair<unsigned int, int>> ChOpenDriveNetwork::GetMarkedLanes() const {
    std::vector<std::pair<unsigned int, int>> out;
    out.reserve(m_markings.size());
    for (const auto& kv : m_markings)
        out.push_back(kv.first);

    return out;
}

// -----------------------------------------------------------------------------------------------
// Network topology
// -----------------------------------------------------------------------------------------------

unsigned int ChOpenDriveNetwork::GetNumRoads() const {
    if (!m_initialized)
        return 0;

    int n = RM_GetNumberOfRoads();
    return n < 0 ? 0 : static_cast<unsigned int>(n);
}

std::vector<unsigned int> ChOpenDriveNetwork::GetRoadIds() const {
    std::vector<unsigned int> ids;
    unsigned int n = GetNumRoads();
    ids.reserve(n);
    for (unsigned int i = 0; i < n; i++)
        ids.push_back(RM_GetIdOfRoadFromIndex(i));

    return ids;
}

double ChOpenDriveNetwork::GetRoadLength(unsigned int road_id) const {
    if (!m_initialized)
        return 0;

    return RM_GetRoadLength(road_id);
}

std::vector<int> ChOpenDriveNetwork::GetLaneIds(unsigned int road_id, double s, int lane_type_mask) const {
    std::vector<int> lane_ids;
    if (!m_initialized)
        return lane_ids;

    int n = RM_GetRoadNumberOfLanes(road_id, s, lane_type_mask);
    if (n <= 0)
        return lane_ids;

    lane_ids.reserve(n);
    for (int i = 0; i < n; i++) {
        int lane_id = 0;
        if (RM_GetLaneIdByIndex(road_id, i, s, lane_type_mask, &lane_id) == kRmOk)
            lane_ids.push_back(lane_id);
    }

    return lane_ids;
}

double ChOpenDriveNetwork::GetLaneWidth(unsigned int road_id, int lane_id, double s) const {
    if (!m_initialized)
        return 0;

    double width = 0;
    if (RM_GetLaneWidthByRoadId(road_id, lane_id, s, &width) != kRmOk)
        return 0;

    return width;
}

// -----------------------------------------------------------------------------------------------
// Coordinate conversion
// -----------------------------------------------------------------------------------------------

bool ChOpenDriveNetwork::LaneToWorld(const ChLaneCoord& lane_pos, ChCoordsys<>& pose, bool align_to_road) const {
    pose = CSYSNORM;

    if (!m_initialized)
        return false;

    if (RmFailed(RM_SetLanePosition(m_scratch, lane_pos.road_id, lane_pos.lane_id, lane_pos.offset,
                                    lane_pos.s, true)))
        return false;

    RM_PositionData data;
    if (RmFailed(RM_GetPositionData(m_scratch, &data)))
        return false;

    ChVector3d pos = ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z));

    // Yaw-pitch-roll about the ISO axes, matching CRGTerrain's use of a plain heading rotation.
    ChQuaterniond rot = QuatFromAngleZ(data.h);
    if (align_to_road)
        rot = rot * QuatFromAngleY(data.p) * QuatFromAngleX(data.r);

    pose = ChCoordsys<>(pos, rot);
    return true;
}

ChCoordsys<> ChOpenDriveNetwork::LaneToWorld(const ChLaneCoord& lane_pos, bool align_to_road) const {
    ChCoordsys<> pose;
    if (!LaneToWorld(lane_pos, pose, align_to_road)) {
        std::cerr << "ChOpenDriveNetwork::LaneToWorld(): no such lane position (road "
                  << lane_pos.road_id << ", lane " << lane_pos.lane_id << ", s " << lane_pos.s << ")"
                  << std::endl;
    }

    return pose;
}

bool ChOpenDriveNetwork::RoadToWorld(unsigned int road_id, double s, double t, ChCoordsys<>& pose) const {
    pose = CSYSNORM;

    if (!m_initialized)
        return false;

    if (RmFailed(RM_SetRoadPosition(m_scratch, road_id, s, t, true)))
        return false;

    RM_PositionData data;
    if (RmFailed(RM_GetPositionData(m_scratch, &data)))
        return false;

    pose = ChCoordsys<>(ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z)), QuatFromAngleZ(data.h));
    return true;
}

bool ChOpenDriveNetwork::SetScratchFromWorld(const ChVector3d& loc, double heading) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    return !RmFailed(RM_SetWorldXYZHPosition(m_scratch, loc_ISO.x(), loc_ISO.y(), loc_ISO.z(), heading));
}

bool ChOpenDriveNetwork::WorldToLane(const ChVector3d& loc, double heading, ChLaneCoord& lane_pos) const {
    if (!m_initialized || !SetScratchFromWorld(loc, heading))
        return false;

    RM_PositionData data;
    if (RmFailed(RM_GetPositionData(m_scratch, &data)))
        return false;

    lane_pos.road_id = data.roadId;
    lane_pos.lane_id = data.laneId;
    lane_pos.s = data.s;
    lane_pos.offset = data.laneOffset;
    return true;
}

ChLaneInfo ChOpenDriveNetwork::GetLaneInfo(const ChVector3d& loc, double heading, double lookahead) const {
    ChLaneInfo info;
    if (!m_initialized || !SetScratchFromWorld(loc, heading))
        return info;

    RM_RoadLaneInfo data;
    if (RmFailed(RM_GetLaneInfo(m_scratch, lookahead, &data, kLookAheadAtLaneCenter, true)))
        return info;

    info.valid = true;
    info.road_id = data.roadId;
    info.lane_id = data.laneId;
    // esmini reports an unsigned sentinel for "not in a junction"; normalize to -1.
    info.junction_id = (data.junctionId == static_cast<decltype(data.junctionId)>(-1))
                           ? -1
                           : static_cast<int>(data.junctionId);
    info.lane_type = data.lane_type;
    info.s = data.s;
    info.t = data.t;
    info.lane_offset = data.laneOffset;
    info.lane_width = data.width;
    info.heading = data.heading;
    info.pitch = data.pitch;
    info.roll = data.roll;
    info.curvature = data.curvature;
    info.speed_limit = data.speed_limit;
    return info;
}

// -----------------------------------------------------------------------------------------------
// Geometry
// -----------------------------------------------------------------------------------------------

bool ChOpenDriveNetwork::GetElevation(const ChVector3d& loc, double& elevation) const {
    if (!m_initialized)
        return false;

    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);

    // Deliberately the XY form, not XYZH. esmini treats a supplied z as an offset the position
    // carries above the road and hands it straight back, so RM_SetWorldXYZHPosition would report
    // the query point's own elevation -- which as a terrain height means the tires never find
    // ground. The XY form is the one that computes z from the road geometry.
    if (RmFailed(RM_SetWorldXYHPosition(m_scratch_elev, loc_ISO.x(), loc_ISO.y(), 0.0)))
        return false;

    // esmini snaps to the nearest road however far away it is, so a successful call does not by
    // itself mean the query point is over the road. A lane type of NONE is how it reports that.
    if (RM_GetInLaneType(m_scratch_elev) == ChLaneType::NONE)
        return false;

    RM_PositionData data;
    if (RmFailed(RM_GetPositionData(m_scratch_elev, &data)))
        return false;

    elevation = ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z)).z();
    return true;
}

std::vector<ChVector3d> ChOpenDriveNetwork::SampleLaneCenter(unsigned int road_id,
                                                             int lane_id,
                                                             double ds) const {
    std::vector<ChVector3d> points;
    if (!m_initialized || ds <= 0)
        return points;

    double length = GetRoadLength(road_id);
    if (length <= 0)
        return points;

    int num_steps = static_cast<int>(std::ceil(length / ds));
    points.reserve(num_steps + 1);

    for (int i = 0; i <= num_steps; i++) {
        // Clamp the last sample to the road end so the path spans the full road.
        double s = std::min(i * ds, length);

        if (RmFailed(RM_SetLanePosition(m_scratch, road_id, lane_id, 0.0, s, true)))
            continue;

        RM_PositionData data;
        if (RmFailed(RM_GetPositionData(m_scratch, &data)))
            continue;

        points.push_back(ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z)));
    }

    return points;
}

std::vector<ChVector3d> ChOpenDriveNetwork::SampleRoute(const ChLaneCoord& start,
                                                        double distance,
                                                        double junction_choice,
                                                        double ds) const {
    std::vector<ChVector3d> points;
    if (!m_initialized || ds <= 0 || distance <= 0)
        return points;

    // align=true orients the position along the lane's driving direction, which is what makes
    // "forward" well defined for lanes on either side of the reference line.
    if (RmFailed(RM_SetLanePosition(m_scratch, start.road_id, start.lane_id, start.offset, start.s, true))) {
        std::cerr << "ChOpenDriveNetwork::SampleRoute(): no such start position (road "
                  << start.road_id << ", lane " << start.lane_id << ", s " << start.s << ")"
                  << std::endl;
        return points;
    }

    RM_PositionData data;
    if (!RmFailed(RM_GetPositionData(m_scratch, &data)))
        points.push_back(ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z)));

    int num_steps = static_cast<int>(std::ceil(distance / ds));
    points.reserve(num_steps + 1);

    for (int i = 0; i < num_steps; i++) {
        // Stops reporting success once the route leaves the network, which is the natural end
        // of the route rather than an error.
        if (RmFailed(RM_PositionMoveForward(m_scratch, ds, junction_choice)))
            break;
        if (RmFailed(RM_GetPositionData(m_scratch, &data)))
            break;

        points.push_back(ChWorldFrame::FromISO(ChVector3d(data.x, data.y, data.z)));
    }

    return points;
}

std::shared_ptr<ChBezierCurve> ChOpenDriveNetwork::CreateRoutePath(const ChLaneCoord& start,
                                                                   double distance,
                                                                   double junction_choice,
                                                                   double ds) const {
    auto points = SampleRoute(start, distance, junction_choice, ds);

    if (points.size() < 2) {
        std::cerr << "ChOpenDriveNetwork::CreateRoutePath(): could not sample a route from road "
                  << start.road_id << " lane " << start.lane_id << std::endl;
        return nullptr;
    }

    return std::make_shared<ChBezierCurve>(points);
}

std::shared_ptr<ChBezierCurve> ChOpenDriveNetwork::CreateLaneCenterPath(unsigned int road_id,
                                                                       int lane_id,
                                                                       double ds) const {
    auto points = SampleLaneCenter(road_id, lane_id, ds);

    // ChBezierCurve fits tangents through the samples, so it needs at least two knots.
    if (points.size() < 2) {
        std::cerr << "ChOpenDriveNetwork::CreateLaneCenterPath(): could not sample lane " << lane_id
                  << " of road " << road_id << std::endl;
        return nullptr;
    }

    return std::make_shared<ChBezierCurve>(points);
}

void ChOpenDriveNetwork::SetSnapLaneTypes(int lane_type_mask) {
    if (!m_initialized)
        return;

    RM_SetSnapLaneTypes(m_scratch, lane_type_mask);
}

}  // end namespace scenario
}  // end namespace chrono
