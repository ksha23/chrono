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
// Backed by esmini's scenario engine (esminiLib). Chrono owns the ego vehicle's
// dynamics; esmini owns the ambient traffic and the scenario's story.
//
// =============================================================================

#ifndef CH_SCENARIO_PLAYER_H
#define CH_SCENARIO_PLAYER_H

#include <string>
#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChVector3.h"

#include "chrono_scenario/ChApiScenario.h"

namespace chrono {
namespace scenario {

/// @addtogroup scenario_module
/// @{

/// State of one scenario actor, as reported by the scenario engine.
struct ChScenarioActor {
    int id = -1;              ///< scenario object ID
    std::string name;         ///< scenario object name

    ChCoordsys<> pose;        ///< world pose (position and orientation)
    double speed = 0;         ///< longitudinal speed [m/s]

    double length = 0;        ///< bounding box length
    double width = 0;         ///< bounding box width
    double height = 0;        ///< bounding box height

    /// Bounding box center relative to the object reference point, in the object's frame.
    ChVector3d center_offset;

    int object_type = 0;      ///< OpenSCENARIO object type
    int object_category = 0;  ///< OpenSCENARIO object category

    unsigned int road_id = 0;  ///< road ID
    int lane_id = 0;           ///< lane ID
    double s = 0;              ///< longitudinal road coordinate
    double lane_offset = 0;    ///< lateral offset from lane center
};

/// Driving scenario loaded from an ASAM OpenSCENARIO file, run in co-simulation with Chrono.
///
/// The intended loop, per control step, is:
/// \code
///   player.ReportEgoState(ego_pose, ego_speed);  // hand Chrono's ego state to the scenario
///   player.Advance(step);                        // advance the story and the ambient traffic
///   for (const auto& a : player.GetActors()) ... // pull traffic back into Chrono
/// \endcode
///
/// \par Two road managers
/// esminiLib and esminiRMLib are independent shared libraries, each with its own copy of the
/// road manager. A ChScenarioPlayer and a ChOpenDriveNetwork therefore hold *separate* parses of
/// the same OpenDRIVE file. This is harmless -- they agree, being built from one file -- but it
/// does mean the network must be initialized explicitly, conventionally from GetOdrFilename().
///
/// \par Single instance
/// esminiLib keeps the loaded scenario in process-global state, so only one ChScenarioPlayer may
/// be initialized at a time.
class ChApiScenario ChScenarioPlayer {
  public:
    ChScenarioPlayer();
    ~ChScenarioPlayer();

    ChScenarioPlayer(const ChScenarioPlayer&) = delete;
    ChScenarioPlayer& operator=(const ChScenarioPlayer&) = delete;

    /// Disable controllers assigned in the .xosc file (default: false).
    /// Leave this off when the scenario assigns controllers to ambient traffic that should still
    /// run. Reporting an ego state each step overrides the ego's own motion regardless.
    /// Must be called before Initialize.
    void SetDisableControllers(bool val) { m_disable_ctrls = val; }

    /// Enable esmini's .dat recording for later playback (default: false).
    /// Must be called before Initialize.
    void SetRecording(bool val) { m_record = val; }

    /// Load a scenario from the specified .xosc file.
    /// `ego_index` selects which scenario object Chrono drives, by index in the entity list;
    /// the first entity is the conventional ego. Returns false on failure.
    bool Initialize(const std::string& xosc_file, int ego_index = 0);

    /// Return true if a scenario is currently loaded.
    bool IsInitialized() const { return m_initialized; }

    /// Get the path of the OpenDRIVE file this scenario references.
    /// Use this to initialize the matching ChOpenDriveNetwork and ChOpenDriveTerrain.
    std::string GetOdrFilename() const;

    /// Get the scenario object ID of the ego vehicle.
    int GetEgoId() const { return m_ego_id; }

    // --- Co-simulation --------------------------------------------------------------------

    /// Report the Chrono-computed ego state to the scenario engine.
    /// Call once per control step, before Advance, so that scenario triggers relative to the ego
    /// (time-to-collision, relative distance) evaluate against the true dynamics.
    bool ReportEgoState(const ChCoordsys<>& pose, double speed);

    /// Advance the scenario by the given step.
    void Advance(double step);

    /// Get the current scenario time.
    double GetTime() const;

    /// Return true once the scenario has reached its stop trigger.
    bool IsComplete() const;

    // --- Actors ---------------------------------------------------------------------------

    /// Get the number of objects in the scenario, including the ego.
    int GetNumObjects() const;

    /// Get the states of all scenario actors except the ego.
    std::vector<ChScenarioActor> GetActors() const;

    /// Get the state of all scenario objects, including the ego.
    std::vector<ChScenarioActor> GetAllObjects() const;

    /// Get the state of a single object by scenario ID. `actor.id` is -1 if not found.
    ChScenarioActor GetObject(int object_id) const;

  private:
    bool m_initialized;   ///< a scenario is currently loaded
    bool m_disable_ctrls; ///< disable controllers assigned in the .xosc
    bool m_record;        ///< write an esmini recording
    int m_ego_id;         ///< scenario object ID of the ego
    std::string m_filename;  ///< path of the loaded .xosc file

    /// esminiLib keeps the scenario in global state; guards against a second instance.
    static bool m_instance_live;
};

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
