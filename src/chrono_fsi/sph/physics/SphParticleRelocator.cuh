// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Device utilities for moving SPH particles and BCE markers external to the solver
//
// =============================================================================

#ifndef SPH_PARTICLE_RELOCATOR_CUH
#define SPH_PARTICLE_RELOCATOR_CUH

#include <vector>

#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"
#include "chrono_fsi/sph/physics/SphMarkerType.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

struct FsiDataManager;

class SphParticleRelocator {
  public:
    struct DefaultProperties {
        Real rho0;
        Real mu0;
        /// Reset the MCC hardening state of relocated SPH particles to the virgin state set with
        /// SetVirginMccState. Only meaningful when the CRM rheology is MCC (no other rheology
        /// populates that array); ignored otherwise.
        bool reset_pcEvSv;
    };

    SphParticleRelocator(FsiDataManager& data_mgr, const DefaultProperties& props);
    ~SphParticleRelocator() {}

    /// Set the virgin MCC hardening state given to relocated SPH particles, tabulated by destination
    /// height. Entry j is (preconsolidation pressure, volumetric strain rate, specific volume) for a
    /// particle relocated to world height z0 + j*dz; a particle takes the nearest entry, clamped to
    /// the ends of the table. A single value will not do: the state is a function of confinement and
    /// therefore of depth, by up to two orders of magnitude across a patch.
    /// Must be set (non-empty) before relocating SPH particles if reset_pcEvSv is on.
    void SetVirginMccState(Real z0, Real dz, const std::vector<Real3>& states);

    /// Shift all particles of specified type by the given vector.
    /// Properties (density and pressure) of relocated particles are overwritten with the specified values.
    void Shift(MarkerType type, const Real3& shift);

    /// Move all particles of specified type that are currently inside the source AABB to the given AABB.
    /// The destination AABB is assumed to be given in integer grid coordinates. Properties (density and pressure) of
    /// relocated particles are overwritten with the specified values.
    void MoveAABB2AABB(MarkerType type, const RealAABB& aabb_src, const IntAABB& aabb_dest, Real spacing);

  private:
    FsiDataManager& m_data_mgr;         ///< FSI data manager
    DefaultProperties m_props;          ///< particle density and pressure after relocation
    std::vector<Real3> m_mcc_states;    ///< virgin MCC hardening state, one entry per destination layer
    Real m_mcc_z0;                      ///< world height of m_mcc_states[0]
    Real m_mcc_dz;                      ///< height step between consecutive entries of m_mcc_states
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
