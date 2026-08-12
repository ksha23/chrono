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
// =============================================================================

#include "chrono_vehicle/terrain/ChMeshTerrain.h"

namespace chrono {
namespace vehicle {

ChMeshTerrain::ChMeshTerrain(std::shared_ptr<ChMeshHeightField> field, float friction)
    : m_field(field), m_friction(friction), m_outside_height(0) {}

double ChMeshTerrain::GetHeight(const ChVector3d& loc) const {
    if (!m_field || !m_field->IsReady())
        return m_outside_height;

    // The query point's own height matters. A tire contact patch stands on the highest surface
    // at or below it; ignoring z would let a tree canopy or a sign gantry overhead answer as
    // ground, which is the failure mode that makes "include everything" unsafe otherwise.
    double z;
    if (m_field->HeightBelow(loc.x(), loc.y(), loc.z(), z))
        return z;
    return m_outside_height;
}

ChVector3d ChMeshTerrain::GetPoint(const ChVector3d& loc) const {
    return ChVector3d(loc.x(), loc.y(), GetHeight(loc));
}

ChVector3d ChMeshTerrain::GetNormal(const ChVector3d& loc) const {
    if (!m_field || !m_field->IsReady())
        return ChVector3d(0, 0, 1);

    double z;
    ChVector3d normal;
    if (m_field->HeightBelow(loc.x(), loc.y(), loc.z(), z, &normal))
        return normal;
    return ChVector3d(0, 0, 1);
}

float ChMeshTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

}  // end namespace vehicle
}  // end namespace chrono
