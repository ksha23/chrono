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
// Static 3D scenery loaded from an instanced placement manifest.
//
// A road network describes lanes; it does not describe what the world looks
// like. Buildings, poles, signal heads, barriers and street furniture come from
// somewhere else, and they arrive as a large number of placements drawing on a
// much smaller set of meshes -- Mcity, for instance, places 792 instances from
// 139 distinct assets.
//
// This loads exactly that: a manifest naming the meshes and the transforms they
// appear at. Each mesh becomes one visual shape, added repeatedly to a body at
// different frames, so geometry is stored once no matter how often it appears.
// Bodies are fixed and carry no collision geometry: scenery is scenery, and the
// driving surface comes from ChOpenDriveTerrain, which answers contact queries
// analytically rather than against a tessellation.
//
// The manifest is deliberately not tied to any authoring tool. Anything able to
// emit a mesh list and a transform list can feed this; see
// demos_live/mcity/usd_to_chrono.py for a USD producer.
//
// =============================================================================

#ifndef CH_SCENERY_MODEL_H
#define CH_SCENERY_MODEL_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_scenario/ChApiScenario.h"

namespace chrono {
namespace scenario {

/// @addtogroup scenario_module
/// @{

/// What to load from a scenery manifest, and how much of it.
struct ChSceneryOptions {
    /// Groups to include; empty means every group in the manifest.
    /// Groups come from the manifest and are whatever the producer chose -- for Mcity,
    /// "Static", "TrafficPoles", "StreetLights", "Terrain", "TrafficLightCables".
    std::vector<std::string> include_groups;

    /// Groups to skip. Applied after include_groups. Useful for dropping the heaviest
    /// content -- vegetation and perimeter fencing dominate triangle counts while
    /// contributing least to a driving view.
    std::vector<std::string> exclude_groups;

    /// Skip any single asset with more than this many triangles (0 = no limit).
    /// A blunt but effective way to shed a handful of very heavy props.
    unsigned int max_triangles_per_asset = 0;

    /// Group instances onto one body per manifest group rather than a single body for the
    /// whole scene. Costs a handful of extra bodies and makes it possible to hide or move a
    /// category at run time.
    bool body_per_group = true;
};

/// Static scenery: many placements drawn from a small set of meshes.
///
/// \par Manifest format
/// \code
/// {
///   "name": "Mcity",
///   "assets":    [ { "name": "SM_Pole", "mesh": "assets/SM_Pole.obj",
///                    "colour": [0.4, 0.4, 0.4] } ],
///   "instances": [ { "asset": 0, "group": "TrafficPoles",
///                    "pos": [x, y, z], "rot": [w, x, y, z], "scale": [sx, sy, sz] } ]
/// }
/// \endcode
/// Mesh paths are relative to the manifest. Lengths are metres, and the frame is Chrono's own
/// (Z up), so a producer is responsible for any unit or axis conversion.
class ChApiScenario ChSceneryModel {
  public:
    using Options = ChSceneryOptions;

    ChSceneryModel();

    /// Load a manifest and add its geometry to the system.
    /// Returns false only if the manifest itself could not be read; individual assets that are
    /// missing or unreadable are skipped and counted, since a partially fetched asset set should
    /// still give a usable scene.
    bool Load(ChSystem& sys, const std::string& manifest_file, const Options& options = Options());

    /// Bodies carrying the scenery, one per group or one in total.
    const std::vector<std::shared_ptr<ChBody>>& GetBodies() const { return m_bodies; }

    /// Distinct meshes actually loaded.
    unsigned int GetNumAssets() const { return m_num_assets; }

    /// Placements emitted.
    unsigned int GetNumInstances() const { return m_num_instances; }

    /// Instances skipped because their group was filtered out or their asset was unusable.
    unsigned int GetNumSkipped() const { return m_num_skipped; }

    /// Assets named in the manifest whose mesh file could not be found.
    unsigned int GetNumMissingAssets() const { return m_num_missing; }

    /// Print a short summary of what was loaded, per group.
    void ReportTo(std::ostream& out) const;

  private:
    std::vector<std::shared_ptr<ChBody>> m_bodies;
    std::map<std::string, unsigned int> m_per_group;  ///< instances loaded, by group

    unsigned int m_num_assets;
    unsigned int m_num_instances;
    unsigned int m_num_skipped;
    unsigned int m_num_missing;
};

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
