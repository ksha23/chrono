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
// Device ulities for moving SPH particles and BCE markers external to the solver
//
// =============================================================================

////#define DEBUG_LOG

#include <cstdio>

#include <thrust/execution_policy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/count.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/functional.h>
#include <thrust/transform.h>
#include <thrust/partition.h>
#include <thrust/zip_function.h>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/physics/SphParticleRelocator.cuh"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

SphParticleRelocator::SphParticleRelocator(FsiDataManager& data_mgr, const DefaultProperties& props)
    : m_data_mgr(data_mgr), m_props(props), m_mcc_z0(0), m_mcc_dz(1) {}

void SphParticleRelocator::SetVirginMccState(Real z0, Real dz, const std::vector<Real3>& states) {
    ChAssertAlways(dz > 0);
    m_mcc_z0 = z0;
    m_mcc_dz = dz;
    m_mcc_states = states;
}

// Device-side view of the tabulated virgin MCC hardening state (SetVirginMccState). An empty table
// (n == 0) means "leave pcEvSv alone", which is the case for every rheology other than MCC and for
// BCE markers (which never accumulate a hardening state - EulerStep_D skips them - and keep the
// sentinel value assigned at creation).
struct mcc_table {
    const Real3* v;
    int n;
    Real z0;
    Real inv_dz;

    __device__ Real3 at(Real z) const {
        int j = (int)floor((z - z0) * inv_dz + Real(0.5));
        j = min(max(j, 0), n - 1);
        return v[j];
    }
};

// Upload the virgin-state table and build the device view. `storage` must outlive the view.
static mcc_table MakeMccTable(bool enable, const std::vector<Real3>& states, Real z0, Real dz, thrust::device_vector<Real3>& storage) {
    mcc_table t;
    t.v = nullptr;
    t.n = 0;
    t.z0 = z0;
    t.inv_dz = Real(1) / dz;
    if (!enable)
        return t;

    // Relocating SPH particles under MCC without a virgin state would write an uninitialized (or
    // stale) hardening state into every one of them; refuse rather than do that silently.
    ChAssertAlways(!states.empty());
    storage.assign(states.begin(), states.end());
    t.v = mR3CAST(storage);
    t.n = (int)storage.size();
    return t;
}

// Relocation function to shift marker position by a given vector.
// Implements a Thrust unary function to be used with thrust::for_each.
struct shift_op {
    shift_op(const Real3& shift, const SphParticleRelocator::DefaultProperties& props, const mcc_table& mcc) : s(shift), p(props), m(mcc) {}

    template <typename T>
    __device__ void operator()(const T& a) const {
        // Modify position
        Real4 posw = thrust::get<0>(a);
        Real3 pos = mR3(posw);
        pos += s;
        thrust::get<0>(a) = mR4(pos, posw.w);

        // Reset all other marker properties
        Real3 zero = mR3(0);
        thrust::get<1>(a) = zero;                                        // velocity
        thrust::get<2>(a) = mR4(p.rho0, 0, p.mu0, thrust::get<2>(a).w);  // rho, pres, mu, type
        thrust::get<3>(a) = zero;                                        // tau diagonal
        thrust::get<4>(a) = zero;                                        // tau off-diagonal

        // Element 5 is the MCC hardening state; see togrid_op for why it has to be reset with the rest.
        if (m.n > 0)
            thrust::get<5>(a) = m.at(pos.z);
    }

    Real3 s;
    SphParticleRelocator::DefaultProperties p;
    mcc_table m;
};

void SphParticleRelocator::Shift(MarkerType type, const Real3& shift) {
    // Get start and end indices in marker data vectors based on specified type
    int start_idx = 0;
    int end_idx = 0;
    switch (type) {
        case MarkerType::BCE_WALL:
            start_idx = (int)m_data_mgr.countersH->startBoundaryMarkers;
            end_idx = start_idx + (int)m_data_mgr.countersH->numBoundaryMarkers;
            break;
        case MarkerType::SPH_PARTICLE:
            start_idx = 0;
            end_idx = start_idx + (int)m_data_mgr.countersH->numFluidMarkers;
            break;
    }

    thrust::device_vector<Real3> mcc_storage;
    auto mcc = MakeMccTable(m_props.reset_pcEvSv && type == MarkerType::SPH_PARTICLE, m_mcc_states, m_mcc_z0, m_mcc_dz, mcc_storage);

    // Transform all markers in the specified range
    thrust::for_each(m_data_mgr.sphMarkers_D->iterator(start_idx), m_data_mgr.sphMarkers_D->iterator(end_idx), shift_op(shift, m_props, mcc));
}

// Selector function to find particles in a given AABB.
// Implements a Thrust predicate to be used with thrust::transform_if or thrust::partition.
struct inaabb_op {
    inaabb_op(const RealAABB& aabb_src) : aabb(aabb_src) {}

    template <typename T>
    __device__ bool operator()(const T& a) const {
        Real4 posw = thrust::get<0>(a);
        Real3 pos = mR3(posw);
        if (pos.x < aabb.min.x || pos.x > aabb.max.x)
            return false;
        if (pos.y < aabb.min.y || pos.y > aabb.max.y)
            return false;
        if (pos.z < aabb.min.z || pos.z > aabb.max.z)
            return false;
        return true;
    }

    RealAABB aabb;
};

// Relocation function to move particles to a given integer AABB.
// Implements a Thrust unary function to be used with thrust::for_each.
// Operates on a tuple {index, data_tuple}.
struct togrid_op {
    togrid_op(const IntAABB& aabb_dest, Real spacing, const SphParticleRelocator::DefaultProperties& props, const mcc_table& mcc)
        : aabb(aabb_dest), delta(spacing), p(props), m(mcc) {}

    template <typename T>
    __device__ T operator()(const T& t) const {
        int index = thrust::get<0>(t);
        auto a = thrust::get<1>(t);

        // 1. Convert linear index to 3D grid coordinates in an AABB of same size as destination AABB
        int idx = index;
        auto dim = aabb.max - aabb.min;
        int x = idx % (dim.x + 1);
        idx /= (dim.x + 1);
        int y = idx % (dim.y + 1);
        idx /= (dim.y + 1);
        int z = idx;

        // 2. Shift marker grid coordinates to current destination AABB
        x += aabb.min.x;
        y += aabb.min.y;
        z += aabb.min.z;

        // Modify marker position in real coordinates (preserve marker type)
        auto w = thrust::get<0>(a).w;
        Real3 pos = mR3(delta * x, delta * y, delta * z);
        thrust::get<0>(a) = mR4(pos, w);

        // Reset all other marker properties
        Real3 zero = mR3(0);
        thrust::get<1>(a) = zero;                                        // velocity
        thrust::get<2>(a) = mR4(p.rho0, 0, p.mu0, thrust::get<2>(a).w);  // rho, pres, mu, type
        thrust::get<3>(a) = zero;                                        // tau diagonal
        thrust::get<4>(a) = zero;                                        // tau off-diagonal

        // Element 5 is the MCC hardening state. It must be reset with everything else: a relocated
        // marker is virgin material, and leaving the preconsolidation pressure of the soil it used to
        // be gives a particle with zero stress but a deformation history, which is not a state the
        // model can be in. Only reset under MCC -- other rheologies do not allocate this array.
        // The virgin state depends on confinement, hence on the destination layer, so it is looked up
        // by destination height rather than being a single value for the whole patch.
        if (m.n > 0)
            thrust::get<5>(a) = m.at(pos.z);

        return t;
    }

    IntAABB aabb;
    Real delta;
    SphParticleRelocator::DefaultProperties p;
    mcc_table m;
};

void SphParticleRelocator::MoveAABB2AABB(MarkerType type, const RealAABB& aabb_src, const IntAABB& aabb_dest, Real spacing) {
    // Get start and end indices in marker data vectors based on specified type
    int start_idx = 0;
    int end_idx = 0;
    switch (type) {
        case MarkerType::BCE_WALL:
            start_idx = (int)m_data_mgr.countersH->startBoundaryMarkers;
            end_idx = start_idx + (int)m_data_mgr.countersH->numBoundaryMarkers;
            break;
        case MarkerType::SPH_PARTICLE:
            start_idx = 0;
            end_idx = start_idx + (int)m_data_mgr.countersH->numFluidMarkers;
            break;
    }

    // Move markers to be relocated at beginning of data structure
    auto middle = thrust::partition(m_data_mgr.sphMarkers_D->iterator(start_idx), m_data_mgr.sphMarkers_D->iterator(end_idx), inaabb_op(aabb_src));

    auto n_move = (int)(middle - m_data_mgr.sphMarkers_D->iterator(start_idx));

    ChDebugLog("Num candidate markers: " << m_data_mgr.sphMarkers_D->iterator(end_idx) - m_data_mgr.sphMarkers_D->iterator(start_idx));
    ChDebugLog("Num moved markers:     " << n_move);

    // Relocate markers based on their index
    thrust::counting_iterator<uint> idx_first(0);
    thrust::counting_iterator<uint> idx_last = idx_first + n_move;

    auto data_first = m_data_mgr.sphMarkers_D->iterator(start_idx);
    auto data_last = m_data_mgr.sphMarkers_D->iterator(start_idx + n_move);

    thrust::device_vector<Real3> mcc_storage;
    auto mcc = MakeMccTable(m_props.reset_pcEvSv && type == MarkerType::SPH_PARTICLE, m_mcc_states, m_mcc_z0, m_mcc_dz, mcc_storage);

    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(idx_first, data_first)), thrust::make_zip_iterator(thrust::make_tuple(idx_last, data_last)),
                     togrid_op(aabb_dest, spacing, m_props, mcc));
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
