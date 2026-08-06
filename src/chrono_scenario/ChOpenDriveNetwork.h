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
// Backed by esmini's road manager library (esminiRMLib), which is linked as a
// plain C shared library. See https://github.com/esmini/esmini
//
// OpenDRIVE uses the ISO 8855 convention (x forward, y left, z up), the same as
// Chrono's ISO world frame, so positions are mapped through ChWorldFrame exactly
// as CRGTerrain does. Headings are used directly as rotations about the vertical
// axis, again matching CRGTerrain.
//
// =============================================================================

#ifndef CH_OPENDRIVE_NETWORK_H
#define CH_OPENDRIVE_NETWORK_H

#include <memory>
#include <string>
#include <vector>

#include "chrono/core/ChBezierCurve.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChVector3.h"

#include "chrono_scenario/ChApiScenario.h"

namespace chrono {
namespace scenario {

/// @addtogroup scenario_module
/// @{

/// OpenDRIVE lane types, as a bitmask.
///
/// The bit positions are those of the ASAM OpenDRIVE lane type enumeration and are mirrored here
/// so that callers need neither a magic number nor esmini's internal RoadManager.hpp. Note that
/// esminiRMLib.hpp carries a stale doc comment quoting 1966594 for "any drivable"; the value below
/// is derived from the current enum.
namespace ChLaneType {
enum : int {
    NONE = 1 << 0,
    DRIVING = 1 << 1,
    STOP = 1 << 2,
    SHOULDER = 1 << 3,
    BIKING = 1 << 4,
    SIDEWALK = 1 << 5,
    BORDER = 1 << 6,
    RESTRICTED = 1 << 7,
    PARKING = 1 << 8,
    BIDIRECTIONAL = 1 << 9,
    MEDIAN = 1 << 10,
    ROADWORKS = 1 << 14,
    TRAM = 1 << 15,
    RAIL = 1 << 16,
    ENTRY = 1 << 17,
    EXIT = 1 << 18,
    OFF_RAMP = 1 << 19,
    ON_RAMP = 1 << 20,
    CURB = 1 << 21,
    CONNECTING_RAMP = 1 << 22,

    /// Any lane a vehicle may travel on.
    ANY_DRIVING = DRIVING | BIDIRECTIONAL | ENTRY | EXIT | OFF_RAMP | ON_RAMP | CONNECTING_RAMP,

    /// Any lane belonging to the roadway, drivable or not.
    ANY_ROAD = ANY_DRIVING | RESTRICTED | STOP | SHOULDER | PARKING,

    ANY = -1
};
}  // namespace ChLaneType

/// Lane-relative position in an OpenDRIVE network.
/// This is the coordinate system scenarios are naturally written in: "lane -1 of road 3, 40 m
/// along, 0.5 m right of lane center".
struct ChLaneCoord {
    unsigned int road_id = 0;  ///< OpenDRIVE road ID
    int lane_id = -1;          ///< OpenDRIVE lane ID (negative = right of reference line)
    double s = 0;              ///< longitudinal distance along the road reference line
    double offset = 0;         ///< lateral offset from the lane center (positive = left)
};

/// Road and lane properties at a queried location.
struct ChLaneInfo {
    bool valid = false;         ///< false if the query location could not be snapped to the network

    unsigned int road_id = 0;   ///< OpenDRIVE road ID
    int lane_id = 0;            ///< OpenDRIVE lane ID
    int junction_id = -1;       ///< enclosing junction ID, or -1 if not in a junction
    int lane_type = 0;          ///< OpenDRIVE lane type bitmask

    double s = 0;               ///< longitudinal distance along the road reference line
    double t = 0;               ///< lateral distance from the road reference line
    double lane_offset = 0;     ///< lateral distance from the lane center
    double lane_width = 0;      ///< width of the current lane

    double heading = 0;         ///< road heading [rad]
    double pitch = 0;           ///< road pitch [rad]
    double roll = 0;            ///< road roll (banking) [rad]
    double curvature = 0;       ///< road curvature [1/m]
    double speed_limit = 0;     ///< posted speed limit [m/s], 0 if unspecified

    /// True if the location is in a junction.
    bool InJunction() const { return junction_id >= 0; }
};

/// Road network loaded from an ASAM OpenDRIVE file.
///
/// Provides the lane-referenced query API that scenario authoring needs: convert between world
/// poses and lane coordinates, enumerate lanes, and extract lane center lines as Bezier curves
/// suitable for ChPathFollowerDriver.
///
/// \par Single instance
/// esminiRMLib holds the loaded network in process-global state, so only one ChOpenDriveNetwork
/// may be initialized at a time. Constructing a second one while the first is live fails.
///
/// \par Thread safety
/// Queries mutate a shared esmini position handle and are therefore not safe to call concurrently
/// on the same instance. This matches CRGTerrain, which shares a single OpenCRG contact point.
class ChApiScenario ChOpenDriveNetwork {
  public:
    ChOpenDriveNetwork();
    ~ChOpenDriveNetwork();

    ChOpenDriveNetwork(const ChOpenDriveNetwork&) = delete;
    ChOpenDriveNetwork& operator=(const ChOpenDriveNetwork&) = delete;

    /// Load an OpenDRIVE network from the specified .xodr file.
    /// Returns false if the file could not be loaded or another network is already loaded.
    bool Initialize(const std::string& xodr_file);

    /// Return true if a network is currently loaded.
    bool IsInitialized() const { return m_initialized; }

    /// Get the path of the loaded .xodr file.
    const std::string& GetFilename() const { return m_filename; }

    // --- Network topology -----------------------------------------------------------------

    /// Get the number of roads in the network.
    unsigned int GetNumRoads() const;

    /// Get the IDs of all roads in the network.
    std::vector<unsigned int> GetRoadIds() const;

    /// Get the length of the specified road.
    double GetRoadLength(unsigned int road_id) const;

    /// Get the IDs of the lanes on the specified road at the given longitudinal position,
    /// restricted to the given ChLaneType mask.
    std::vector<int> GetLaneIds(unsigned int road_id,
                                double s,
                                int lane_type_mask = ChLaneType::ANY_DRIVING) const;

    /// Get the width of the specified lane at the given longitudinal position.
    double GetLaneWidth(unsigned int road_id, int lane_id, double s) const;

    // --- Coordinate conversion ------------------------------------------------------------

    /// Convert a lane-relative position to a world pose.
    /// If `align_to_road`, the returned orientation follows the road heading, pitch and roll;
    /// otherwise only the heading is applied. Returns an identity coordsys on failure; use the
    /// overload below where a valid pose at the world origin must be distinguished from failure.
    ChCoordsys<> LaneToWorld(const ChLaneCoord& lane_pos, bool align_to_road = true) const;

    /// Convert a lane-relative position to a world pose, reporting whether the lane position exists.
    bool LaneToWorld(const ChLaneCoord& lane_pos, ChCoordsys<>& pose, bool align_to_road = true) const;

    /// Convert a world location to the nearest lane-relative position.
    /// `heading` disambiguates travel direction; pass the vehicle yaw. Returns false if the
    /// location could not be snapped to the network.
    bool WorldToLane(const ChVector3d& loc, double heading, ChLaneCoord& lane_pos) const;

    /// Query full road and lane properties at a world location.
    /// A positive `lookahead` reports properties that far ahead along the road instead of at
    /// `loc` itself, which is what a lane-following controller wants.
    ChLaneInfo GetLaneInfo(const ChVector3d& loc, double heading, double lookahead = 0) const;

    // --- Geometry -------------------------------------------------------------------------

    /// Get the road surface elevation below the specified world location.
    /// Returns false if the location does not lie over the road surface.
    ///
    /// The elevation is resolved by horizontal position only. esmini's road manager treats a
    /// supplied z as an offset carried above the road rather than as a point to project down
    /// from, so it cannot be used to disambiguate stacked roads: over an overpass this reports
    /// the surface esmini snaps to, which need not be the upper deck.
    bool GetElevation(const ChVector3d& loc, double& elevation) const;

    /// Extract the center line of the specified lane as a Bezier curve.
    /// Sampled every `ds` meters along the road. The result can be handed directly to
    /// ChPathFollowerDriver to make a vehicle track that lane.
    std::shared_ptr<ChBezierCurve> CreateLaneCenterPath(unsigned int road_id,
                                                        int lane_id,
                                                        double ds = 1.0) const;

    /// Sample points along the center line of the specified lane.
    std::vector<ChVector3d> SampleLaneCenter(unsigned int road_id, int lane_id, double ds = 1.0) const;

    /// Restrict which lane types world-to-lane snapping will consider.
    /// The mask follows esmini's roadmanager::Lane::LaneType. Defaults to any drivable lane.
    void SetSnapLaneTypes(int lane_type_mask);

  private:
    /// Populate the internal scratch handle from a world location and heading.
    /// Returns false if the location could not be snapped to the network.
    bool SetScratchFromWorld(const ChVector3d& loc, double heading) const;

    bool m_initialized;    ///< a network is currently loaded
    std::string m_filename;  ///< path of the loaded .xodr file
    int m_scratch;         ///< esmini position handle for lane queries
    /// Separate handle for elevation queries. esmini's position object retains a z offset once an
    /// absolute z is set, so elevation lookups must not share a handle with the lane queries.
    int m_scratch_elev;

    /// esminiRMLib keeps the network in global state; guards against a second instance.
    static bool m_instance_live;
};

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
