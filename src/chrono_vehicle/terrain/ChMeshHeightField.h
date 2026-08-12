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
// A queryable ground height sampled from the geometry that is actually drawn.
//
// Why this exists
// ---------------
// A scene assembled from two sources -- an OpenDRIVE network for the lanes and
// a mesh set for the visuals -- has two different ground surfaces, and they do
// not have to agree. Measured on Mcity, whose .xodr comes from the CARLA export
// and whose meshes come from the Omniverse stage, the analytic OpenDRIVE surface
// sits within a median 17 mm of the rendered carriageway but with a 5th-95th
// percentile spread of -0.24 to +0.29 m and a worst case over a metre. That
// spread is not a datum error and no constant offset removes it: the vehicle
// visibly floats in some places and sinks in others.
//
// Coverage differs too. Roughly 30% of the rendered road surface -- kerbs,
// medians, aprons -- is not a drivable OpenDRIVE lane at all, so an analytic
// surface has nothing to say there.
//
// This class closes both gaps by answering from the drawn triangles, so what the
// tires stand on is what the camera sees, by construction. It is a query
// structure, not a collision shape: no bodies, no contact geometry, no solver
// cost. Triangles are bucketed on a uniform XY grid and a lookup interpolates
// the one triangle under the query point.
//
// OpenDRIVE still does what it is good at -- lanes, routing, markings, signals.
// It just stops being the source of ground height.
//
// =============================================================================

#ifndef CH_MESH_HEIGHT_FIELD_H
#define CH_MESH_HEIGHT_FIELD_H

#include <array>
#include <string>
#include <vector>

#include "chrono/core/ChVector3.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Ground height sampled from a triangle soup, indexed for point queries.
class CH_VEHICLE_API ChMeshHeightField {
  public:
    ChMeshHeightField();

    /// Add one world-space triangle. Call before Build().
    void AddTriangle(const ChVector3d& a, const ChVector3d& b, const ChVector3d& c);

    /// Index the accumulated triangles for querying.
    ///
    /// \a cell_size is the grid pitch in metres. It trades memory against the number of
    /// triangles tested per query; a value near the median triangle size is about right, and
    /// the default suits road-scale geometry.
    void Build(double cell_size = 2.0);

    /// Ground height under a world XY location.
    ///
    /// Returns false when no triangle covers the point. When several do -- an overpass, a kerb
    /// overhanging a gutter, a bridge above a road -- the highest wins, which is the surface
    /// something dropped from above would land on.
    bool Height(double x, double y, double& z, ChVector3d* normal = nullptr) const;

    /// Ground height beneath a specific point, ignoring anything above it.
    ///
    /// The z-aware form, and the one to prefer when the mesh contains more than the ground. A
    /// query at a tyre's contact patch should not be answered by a tree canopy or a sign gantry
    /// forty metres overhead, and without the reference height the "highest wins" rule does
    /// exactly that. \a tolerance allows for a contact point sitting slightly under the surface.
    /// Falls back to the lowest surface above the point when there is nothing below it, so a
    /// vehicle spawned under the road is pushed out rather than left with no ground at all.
    bool HeightBelow(double x, double y, double z_ref, double& z, ChVector3d* normal = nullptr,
                     double tolerance = 0.5) const;

    /// True once Build() has run on a non-empty triangle set.
    bool IsReady() const { return m_ready; }

    size_t GetNumTriangles() const { return m_tris.size(); }

    /// Mean number of triangles a query has to test. Useful for choosing a cell size.
    double GetMeanBucketOccupancy() const;

    /// Extent of the indexed geometry, valid after Build().
    void GetExtent(ChVector3d& lo, ChVector3d& hi) const;

    /// Write the accumulated triangles as a Wavefront mesh.
    ///
    /// For handing the same surface to something that wants a file rather than a query --
    /// RigidTerrain::AddPatch, in particular, which builds a collision mesh from an OBJ. The
    /// triangles are already in world space, so the patch goes in at the identity transform.
    bool WriteWavefront(const std::string& path) const;

  private:
    int CellIndex(int ix, int iy) const { return iy * m_nx + ix; }

    /// Interpolate one triangle at an XY location; false if the point is outside it.
    bool SampleTriangle(int idx, double x, double y, double& z, ChVector3d& n) const;

    std::vector<std::array<ChVector3d, 3>> m_tris;
    std::vector<std::vector<int>> m_cells;  ///< triangle indices per grid cell

    double m_x0, m_y0, m_cell;
    int m_nx, m_ny;
    ChVector3d m_lo, m_hi;
    bool m_ready;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
