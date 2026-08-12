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
// Terrain that answers from the geometry a scene actually draws.
//
// RigidTerrain gives a vehicle a contact surface by putting collision shapes in
// the solver. That is the right answer when the surface must react -- when
// something can push it, or when arbitrary bodies must collide with it. For a
// vehicle driving over static scenery it is more than is needed: the ground is
// never going to move, and a tire only ever asks two questions of it, "how high
// is it here" and "which way does it face".
//
// This answers exactly those two, from a ChMeshHeightField built out of the
// scene's own triangles. No bodies, no collision shapes, no broadphase, no
// solver cost -- and because the query reads the drawn geometry, the surface a
// tire rests on and the surface on screen cannot drift apart.
//
// The limitation is inherent to a height field: one surface per XY. It cannot
// represent a road that passes both over and under a bridge. Queries are
// height-aware, so overhead geometry never answers for the ground beneath it,
// but only one deck of a true overpass is reachable.
//
// =============================================================================

#ifndef CH_MESH_TERRAIN_H
#define CH_MESH_TERRAIN_H

#include <memory>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/ChMeshHeightField.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Terrain defined by a queryable mesh height field.
class CH_VEHICLE_API ChMeshTerrain : public ChTerrain {
  public:
    /// Construct over an already-built height field.
    ChMeshTerrain(std::shared_ptr<ChMeshHeightField> field, float friction = 0.8f);

    /// Height of the surface beneath \a loc.
    /// Anything above the query point is ignored, so a canopy or gantry cannot act as ground.
    virtual double GetHeight(const ChVector3d& loc) const override;

    /// Point on the surface beneath \a loc.
    ///
    /// Must be overridden, not inherited: ChTerrain's default returns (x, y, 0), and the tire
    /// models ask for the point rather than the height. Inheriting it tells every wheel the
    /// ground is at z = 0, which on a site whose datum is 274 m is a silent 274 m hole.
    virtual ChVector3d GetPoint(const ChVector3d& loc) const override;

    /// Surface normal beneath \a loc; vertical where the mesh does not cover it.
    virtual ChVector3d GetNormal(const ChVector3d& loc) const override;

    /// Coefficient of friction, constant unless a functor is installed.
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override;

    /// Height reported where the mesh has no coverage (default: 0).
    void SetOutsideHeight(double height) { m_outside_height = height; }

    void SetContactFrictionCoefficient(float friction) { m_friction = friction; }

    std::shared_ptr<ChMeshHeightField> GetHeightField() const { return m_field; }

  private:
    std::shared_ptr<ChMeshHeightField> m_field;
    float m_friction;
    double m_outside_height;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
