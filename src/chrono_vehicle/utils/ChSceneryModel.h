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
#include "chrono/core/ChVector3.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/terrain/ChMeshHeightField.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
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

    /// Rigid offset applied to every placement.
    ///
    /// Needed when the scenery and the driving surface come from different sources that do not
    /// share a vertical datum, which is common enough to be worth a knob but is not universal.
    /// Mcity happens not to need one: sampling OpenDRIVE elevation against the road mesh in
    /// world space puts the two within a median 0.012 m, so the default is no offset. Measure
    /// before setting this -- comparing against asset-local vertices instead of placed ones
    /// produces a confident, wrong answer.
    ChVector3d position_offset = ChVector3d(0, 0, 0);

    /// Assets whose triangles also become a queryable ground height, matched by name substring.
    ///
    /// Empty means no height field is built. Naming the drivable surfaces here -- for Mcity
    /// "SM_Roads", "SM_Sidewalks", "SM_Curbs", "SM_Ground" -- lets a vehicle stand on the
    /// geometry it is drawn against instead of on a separately authored analytic surface that
    /// need not agree with it. See ChMeshHeightField for the measured disagreement.
    std::vector<std::string> ground_assets;

    /// Sample every asset into the ground height field except those matching ground_exclude.
    ///
    /// Safer as a default than listing what to include: a whitelist silently omits any surface
    /// nobody remembered to name, and the failure mode is a vehicle sinking through geometry it
    /// can see. Overhead objects are not a problem here because the query is height-aware -- see
    /// ChMeshHeightField::HeightBelow -- so the exclusions are only about build cost.
    bool ground_all = false;

    /// Asset name fragments never sampled as ground. Only consulted when ground_all is set.
    std::vector<std::string> ground_exclude;

    /// Manifest groups never sampled as ground. Only consulted when ground_all is set.
    ///
    /// Prefer this to ground_exclude wherever a group exists. Names are a poor filter for
    /// categories: excluding "Tree" does not exclude Red_Maple, Scarlet_Oak or Yew, and the
    /// result was 447 trees in the collision surface -- 20.7M triangles, and a vehicle standing
    /// on foliage. The group is stated by the producer and is exact.
    std::vector<std::string> ground_exclude_groups;

    /// Grid pitch of the height field, in metres.
    double ground_cell_size = 2.0;

    /// Split each group into bodies on an XY grid of this pitch, in metres (0 = one body per
    /// group).
    ///
    /// Bodies are the unit a visual system can switch on and off, so this is what makes
    /// distance-based streaming possible: with a single body per group, "the foliage" is one
    /// indivisible object and gating it is all-or-nothing. Tiling turns a town into a few dozen
    /// independently discardable pieces, at the cost of a few dozen extra collision-free bodies.
    double tile_size = 0.0;
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
class CH_VEHICLE_API ChSceneryModel {
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

    /// Material slots that resolved to a texture on disk.
    unsigned int GetNumTexturedSurfaces() const { return m_num_textures; }

    /// Ground height sampled from the assets named in Options::ground_assets.
    /// Null when none were named.
    std::shared_ptr<ChMeshHeightField> GetGroundHeightField() const { return m_ground; }

    /// Write the ground surface to a Wavefront file, for RigidTerrain::AddPatch.
    /// Returns false if no ground was accumulated or the file could not be written.
    bool WriteGroundMesh(const std::string& path) const {
        return m_ground && m_ground->IsReady() && m_ground->WriteWavefront(path);
    }

    /// Print a short summary of what was loaded, per group.
    void ReportTo(std::ostream& out) const;

  private:
    std::vector<std::shared_ptr<ChBody>> m_bodies;
    std::map<std::string, unsigned int> m_per_group;  ///< instances loaded, by group

    unsigned int m_num_assets;
    unsigned int m_num_instances;
    unsigned int m_num_skipped;
    unsigned int m_num_missing;
    unsigned int m_num_textures;
    std::shared_ptr<ChMeshHeightField> m_ground;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
