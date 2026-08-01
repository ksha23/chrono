// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Class for performing time integration in fluid system.
// =============================================================================

#include <thrust/execution_policy.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/logical.h>

#include "chrono/utils/ChConstants.h"
#include "chrono_fsi/sph/physics/SphFluidDynamics.cuh"
#include "chrono_fsi/sph/physics/SphForceWCSPH.cuh"
#include "chrono_fsi/sph/physics/SphForceISPH.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"
#include "chrono_fsi/sph/utils/SphUtilsLogging.cuh"

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {
namespace sph {

void CopyParametersToDevice_SphFluidDynamics(std::shared_ptr<ChFsiParamsSPH> paramsH, std::shared_ptr<Counters> countersH) {
    gpuMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(ChFsiParamsSPH));
    gpuCheckError();
    gpuMemcpyToSymbolAsync(countersD, countersH.get(), sizeof(Counters));
    gpuCheckError();
}

SphFluidDynamics::SphFluidDynamics(FsiDataManager& data_mgr, SphBceManager& bce_mgr, bool verbose, bool check_errors)
    : m_data_mgr(data_mgr), m_verbose(verbose), m_check_errors(check_errors), m_errflagD(nullptr), m_activity_chunks_valid(false) {
    collisionSystem = chrono_types::make_shared<SphCollisionSystem>(data_mgr);

    if (m_data_mgr.paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH)
        forceSystem = chrono_types::make_shared<SphForceISPH>(data_mgr, bce_mgr, verbose, m_check_errors);
    else
        forceSystem = chrono_types::make_shared<SphForceWCSPH>(data_mgr, bce_mgr, verbose, m_check_errors);

    gpuStreamCreate(&m_copy_stream);
    gpuMallocErrorFlag(m_errflagD);
}

SphFluidDynamics::~SphFluidDynamics() {
    gpuStreamDestroy(m_copy_stream);
    gpuFreeErrorFlag(m_errflagD);
}

// -----------------------------------------------------------------------------

void SphFluidDynamics::Initialize() {
    gpuMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    gpuMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
    gpuMemcpyFromSymbol(m_data_mgr.paramsH.get(), paramsD, sizeof(ChFsiParamsSPH));

    forceSystem->Initialize();
    collisionSystem->Initialize();
}

// -----------------------------------------------------------------------------

void SphFluidDynamics::ProximitySearch() {
    collisionSystem->ArrangeData(m_data_mgr.sphMarkers_D, m_data_mgr.sortedSphMarkers2_D);
    collisionSystem->NeighborSearch(m_data_mgr.sortedSphMarkers2_D);
}

// -----------------------------------------------------------------------------

void SphFluidDynamics::CopySortedMarkers(const std::shared_ptr<SphMarkerDataD>& in, std::shared_ptr<SphMarkerDataD>& out) {
    thrust::copy(in->posRadD.begin(), in->posRadD.begin() + m_data_mgr.countersH->numExtendedParticles, out->posRadD.begin());
    thrust::copy(in->velMasD.begin(), in->velMasD.begin() + m_data_mgr.countersH->numExtendedParticles, out->velMasD.begin());
    thrust::copy(in->rhoPresMuD.begin(), in->rhoPresMuD.begin() + m_data_mgr.countersH->numExtendedParticles, out->rhoPresMuD.begin());
    if (m_data_mgr.paramsH->elastic_SPH) {
        thrust::copy(in->tauXxYyZzD.begin(), in->tauXxYyZzD.end(), out->tauXxYyZzD.begin());
        thrust::copy(in->tauXyXzYzD.begin(), in->tauXyXzYzD.end(), out->tauXyXzYzD.begin());
        thrust::copy(in->pcEvSvD.begin(), in->pcEvSvD.end(), out->pcEvSvD.begin());
    }
}

double SphFluidDynamics::computeTimeStep() const {
    size_t valid_entries = m_data_mgr.countersH->numExtendedParticles;
    double min_courant_viscous_time_step =
        static_cast<double>(thrust::reduce(thrust::device, m_data_mgr.courantViscousTimeStepD.begin(), m_data_mgr.courantViscousTimeStepD.begin() + valid_entries,
                                           std::numeric_limits<Real>::max(), thrust::minimum<Real>()));
    double min_acceleration_time_step =
        static_cast<double>(thrust::reduce(thrust::device, m_data_mgr.accelerationTimeStepD.begin(), m_data_mgr.accelerationTimeStepD.begin() + valid_entries,
                                           std::numeric_limits<Real>::max(), thrust::minimum<Real>()));

    double adjusted_time_step = 0.3 * std::min(min_courant_viscous_time_step, min_acceleration_time_step);
    // Log the time step values for analysis
#ifdef FSI_COUNT_LOGGING_ENABLED
    QuantityLogger::GetInstance().AddValue("time_step", adjusted_time_step);
    QuantityLogger::GetInstance().AddValue("min_courant_viscous_time_step", min_courant_viscous_time_step);
    QuantityLogger::GetInstance().AddValue("min_acceleration_time_step", min_acceleration_time_step);
#endif
    return adjusted_time_step;
}

//// TODO - revisit application of particle shifting (explicit schemes)
////        currently, a new v_XSPH is calculated at every force evaluation and used in the subsequent position update
////        should this be done only once per step?
void SphFluidDynamics::DoStepDynamics(std::shared_ptr<SphMarkerDataD> y, Real t, Real h, IntegrationScheme scheme) {
    switch (scheme) {
        case IntegrationScheme::EULER: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y, h);                     // y <==  y_{n+1} = y_n + h * f(t_n, y_n)
            ApplyBoundaryConditions(y);

            break;
        }

        case IntegrationScheme::RK2: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            auto& y_tmp = m_data_mgr.sortedSphMarkers1_D;
            CopySortedMarkers(y, y_tmp);  // y_tmp <- y_n

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y_tmp, h / 2);             // y_tmp <==  K1 = y_n + (h/2) * f(t_n, y_n)
            ApplyBoundaryConditions(y_tmp);

            forceSystem->ForceSPH(y_tmp, t + h / 2, dummy);  // f(t_n + h/2, K1)
            EulerStep(y, h);                                 // y <== y_{n+1} = y_n + h * f(t_n + h/2, K1)
            ApplyBoundaryConditions(y);

            break;
        }

        case IntegrationScheme::SYMPLECTIC: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            auto& y_tmp = m_data_mgr.sortedSphMarkers1_D;
            CopySortedMarkers(y, y_tmp);  // y_tmp <- y_n

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y_tmp, h / 2);             // y_tmp <== y_{n+1/2} = y_n + (h/2) * f(t_n, y_n)
            ApplyBoundaryConditions(y_tmp);

            forceSystem->ForceSPH(y_tmp, t + h / 2, dummy);  // f_{n+1/2} = f(t_n + h/2, y_{n+1/2})
            MidpointStep(y, h);                              // y_{n+1} = y_n + h * f_{n+1/2}

            break;
        }

        case IntegrationScheme::IMPLICIT_SPH: {
            forceSystem->ForceSPH(y, t, h);
            ApplyBoundaryConditions(y);

            break;
        }
    }
}

// -----------------------------------------------------------------------------

// Decide the activity of a single marker. Shared by the flat and the chunked launcher so the two
// paths cannot drift apart. Writes activityIdentifierD/extendedActivityIdD (and zeroes the velocity
// of a marker that is not active) exactly as the original single-kernel version did.
__device__ inline void EvalMarkerActivity(uint index,
                                          const Real4* posRadD,
                                          Real3* velMasD,
                                          const Real3* pos_bodies_D,
                                          const Real3* pos_nodes1D_D,
                                          const Real3* pos_nodes2D_D,
                                          int32_t* activityIdentifierD,
                                          int32_t* extendedActivityIdD,
                                          bool isFluid,
                                          double time,
                                          Real3& posOut,
                                          int32_t& extendedOut) {
    // Set the particle as an active particle
    int32_t activity = 1;
    int32_t extended = 1;
    Real3 domainDims = paramsD.boxDims;
    Real3 domainOrigin = paramsD.worldOrigin;
    bool x_periodic = paramsD.x_periodic;
    bool y_periodic = paramsD.y_periodic;
    bool z_periodic = paramsD.z_periodic;

    Real3 posRadA = mR3(posRadD[index]);
    if (time >= paramsD.settlingTime) {
        size_t numFsiBodies = countersD.numFsiBodies;
        size_t numFsiNodes1D = countersD.numFsiNodes1D;
        size_t numFsiNodes2D = countersD.numFsiNodes2D;
        size_t numTotal = numFsiBodies + numFsiNodes1D + numFsiNodes2D;

        // Check the activity of this particle
        uint isNotActive = 0;
        uint isNotExtended = 0;
        Real3 Acdomain = paramsD.bodyActiveDomain;
        Real3 ExAcdomain = paramsD.bodyActiveDomain + mR3(2 * paramsD.h_multiplier * paramsD.h);

        for (uint num = 0; num < numFsiBodies; num++) {
            Real3 detPos = posRadA - pos_bodies_D[num];
            if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
                isNotActive = isNotActive + 1;
            if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
                isNotExtended = isNotExtended + 1;
        }

        for (uint num = 0; num < numFsiNodes1D; num++) {
            Real3 detPos = posRadA - pos_nodes1D_D[num];
            if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
                isNotActive = isNotActive + 1;
            if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
                isNotExtended = isNotExtended + 1;
        }

        for (uint num = 0; num < numFsiNodes2D; num++) {
            Real3 detPos = posRadA - pos_nodes2D_D[num];
            if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
                isNotActive = isNotActive + 1;
            if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
                isNotExtended = isNotExtended + 1;
        }

        // Set the particle as an inactive particle if needed
        if (isNotActive == numTotal && numTotal > 0) {
            activity = 0;
            velMasD[index] = mR3(0.0);
        }
        if (isNotExtended == numTotal && numTotal > 0)
            extended = 0;
    }
    // Check if the particle is outside the zombie domain
    if (isFluid) {
        bool outside_domain = false;

        // Check X boundaries - only inactivate if not periodic
        if (!x_periodic && (posRadA.x < domainOrigin.x || posRadA.x > domainOrigin.x + domainDims.x)) {
            outside_domain = true;
        }

        // Check Y boundaries - only inactivate if not periodic
        if (!y_periodic && (posRadA.y < domainOrigin.y || posRadA.y > domainOrigin.y + domainDims.y)) {
            outside_domain = true;
        }

        // Check Z boundaries - only inactivate if not periodic
        if (!z_periodic && (posRadA.z < domainOrigin.z || posRadA.z > domainOrigin.z + domainDims.z)) {
            outside_domain = true;
        }

        if (outside_domain) {
            activity = -1;
            extended = -1;
            velMasD[index] = mR3(0.0);
        }
    }

    activityIdentifierD[index] = activity;
    extendedActivityIdD[index] = extended;

    posOut = posRadA;
    extendedOut = extended;
}

__global__ void UpdateActivityD(const Real4* posRadD,
                                Real3* velMasD,
                                const Real3* pos_bodies_D,
                                const Real3* pos_nodes1D_D,
                                const Real3* pos_nodes2D_D,
                                int32_t* activityIdentifierD,
                                int32_t* extendedActivityIdD,
                                const Real4* rhoPreMuD,
                                double time) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers) {
        return;
    }

    Real3 pos;
    int32_t extended;
    EvalMarkerActivity(index, posRadD, velMasD, pos_bodies_D, pos_nodes1D_D, pos_nodes2D_D, activityIdentifierD, extendedActivityIdD, IsFluidParticle(rhoPreMuD[index].w), time,
                       pos, extended);
}

// -----------------------------------------------------------------------------
// Chunked activity update.
//
// The flat kernel above reads the position of every marker in the world on every step, so its cost
// grows with terrain extent even though the physics does not. The markers that dominate that count
// are, by construction, the ones nowhere near a solid - and a marker that is not in the extended
// active set is not in the sorted arrays at all, so it is never integrated and never moves. That
// makes it possible to remember, per fixed-size block of marker indices ("chunk"):
//   - whether every marker in the chunk was outside the extended active domain last time it was
//     looked at ("dormant"), and
//   - the bounding box of the chunk's marker positions, which stays valid for exactly as long as the
//     chunk is dormant.
// A dormant chunk whose box does not touch the (extended) active region cannot contain a marker
// whose activity would change, so it is skipped without reading a single position.
//
// Anything that moves or reorders markers outside the solver invalidates the cached boxes; the
// caller signals that with force_full.
//
// A chunk is a run of slots in activityOrderD, not a run of marker indices. Marker index order is
// arbitrary - ChFsiProblemSPH emits markers in hash-set iteration order, which spreads consecutive
// indices over the whole world - so index-contiguous chunks would have world-sized boxes and nothing
// could ever be skipped. activityOrderD is the marker list in Morton order, which makes the boxes
// tight without touching the marker arrays themselves (so nothing downstream sees a different
// ordering, and results are unchanged).
#define ACTIVITY_BLOCK_SIZE 256
#define ACTIVITY_CHUNK_SIZE 512

// Spread the low 10 bits of v out with 2-bit gaps, for a 30-bit Morton code.
__device__ inline uint ExpandBits10(uint v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

// Morton key of each marker on a 1024^3 lattice fitted to the computational domain. Per-axis
// normalization means a long thin terrain patch still gets 1024 divisions along its length.
__global__ void ComputeActivityKeysD(const Real4* __restrict__ posRadD, const Real4* __restrict__ rhoPreMuD, uint* __restrict__ keys, uint* __restrict__ order, uint numAllMarkers,
                                     Real3 origin, Real3 invCell) {
    uint i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numAllMarkers)
        return;

    Real3 p = mR3(posRadD[i]);
    int ix = (int)((p.x - origin.x) * invCell.x);
    int iy = (int)((p.y - origin.y) * invCell.y);
    int iz = (int)((p.z - origin.z) * invCell.z);
    ix = min(max(ix, 0), 1023);
    iy = min(max(iy, 0), 1023);
    iz = min(max(iz, 0), 1023);

    keys[i] = (ExpandBits10((uint)ix) << 2) | (ExpandBits10((uint)iy) << 1) | ExpandBits10((uint)iz);
    // Marker type never changes, so cache it in the top bit of the order entry. That entry is read
    // coalesced by the chunked update, which saves it a scattered load of rhoPresMu per marker.
    order[i] = i | (IsFluidParticle(rhoPreMuD[i].w) ? 0x80000000u : 0u);
}

__global__ void UpdateActivityChunkedD(const Real4* posRadD,
                                       Real3* velMasD,
                                       const Real3* pos_bodies_D,
                                       const Real3* pos_nodes1D_D,
                                       const Real3* pos_nodes2D_D,
                                       int32_t* activityIdentifierD,
                                       int32_t* extendedActivityIdD,
                                       double time,
                                       Real3 regionMin,
                                       Real3 regionMax,
                                       Real3* chunkAabbMin,
                                       Real3* chunkAabbMax,
                                       int32_t* chunkDormant,
                                       const uint* __restrict__ activityOrder,
                                       int forceFull) {
    __shared__ Real3 s_lo[ACTIVITY_BLOCK_SIZE];
    __shared__ Real3 s_hi[ACTIVITY_BLOCK_SIZE];
    __shared__ int32_t s_act[ACTIVITY_BLOCK_SIZE];
    __shared__ int s_skip;

    const uint chunk = blockIdx.x;
    const uint tid = threadIdx.x;

    if (tid == 0) {
        int skip = 0;
        if (!forceFull && chunkDormant[chunk]) {
            Real3 lo = chunkAabbMin[chunk];
            Real3 hi = chunkAabbMax[chunk];
            skip = (hi.x < regionMin.x || lo.x > regionMax.x ||  //
                    hi.y < regionMin.y || lo.y > regionMax.y ||  //
                    hi.z < regionMin.z || lo.z > regionMax.z)
                       ? 1
                       : 0;
        }
        s_skip = skip;
    }
    __syncthreads();
    if (s_skip)
        return;

    const Real big = Real(3.4e38);
    Real3 lo = mR3(big);
    Real3 hi = mR3(-big);
    int32_t numExtended = 0;

    for (uint k = tid; k < ACTIVITY_CHUNK_SIZE; k += ACTIVITY_BLOCK_SIZE) {
        uint slot = chunk * ACTIVITY_CHUNK_SIZE + k;
        if (slot >= countersD.numAllMarkers)
            break;
        uint packed = activityOrder[slot];
        uint index = packed & 0x7FFFFFFFu;
        Real3 pos;
        int32_t extended;
        EvalMarkerActivity(index, posRadD, velMasD, pos_bodies_D, pos_nodes1D_D, pos_nodes2D_D, activityIdentifierD, extendedActivityIdD, (packed >> 31) != 0, time, pos, extended);
        lo = mR3(fmin(lo.x, pos.x), fmin(lo.y, pos.y), fmin(lo.z, pos.z));
        hi = mR3(fmax(hi.x, pos.x), fmax(hi.y, pos.y), fmax(hi.z, pos.z));
        numExtended += (extended > 0) ? 1 : 0;
    }

    s_lo[tid] = lo;
    s_hi[tid] = hi;
    s_act[tid] = numExtended;
    __syncthreads();

    for (uint s = ACTIVITY_BLOCK_SIZE / 2; s > 0; s >>= 1) {
        if (tid < s) {
            Real3 a = s_lo[tid], b = s_lo[tid + s];
            s_lo[tid] = mR3(fmin(a.x, b.x), fmin(a.y, b.y), fmin(a.z, b.z));
            Real3 c = s_hi[tid], d = s_hi[tid + s];
            s_hi[tid] = mR3(fmax(c.x, d.x), fmax(c.y, d.y), fmax(c.z, d.z));
            s_act[tid] += s_act[tid + s];
        }
        __syncthreads();
    }

    if (tid == 0) {
        chunkAabbMin[chunk] = s_lo[0];
        chunkAabbMax[chunk] = s_hi[0];
        // A chunk is dormant only if no marker in it is in the *extended* active set. Those markers
        // are absent from the sorted arrays, so nothing in the step can move them or write their
        // state - which is what keeps the cached box above valid while the chunk stays dormant.
        chunkDormant[chunk] = (s_act[0] == 0) ? 1 : 0;
    }
}

void SphFluidDynamics::BuildActivityOrder(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    const auto& paramsH = m_data_mgr.paramsH;
    uint numAllMarkers = (uint)m_data_mgr.countersH->numAllMarkers;

    // Lattice fitted to the computational domain, 1024 divisions per axis.
    Real3 dims = paramsH->boxDims;
    Real3 invCell = mR3(Real(1024) / fmax(dims.x, Real(1e-6)),  //
                        Real(1024) / fmax(dims.y, Real(1e-6)),  //
                        Real(1024) / fmax(dims.z, Real(1e-6)));

    m_data_mgr.activityOrderD.resize(numAllMarkers);
    thrust::device_vector<uint> keys(numAllMarkers);

    uint numBlocks, numThreads;
    computeGridSize(numAllMarkers, 256, numBlocks, numThreads);
    ComputeActivityKeysD<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD), U1CAST(keys), U1CAST(m_data_mgr.activityOrderD), numAllMarkers,
                                                   paramsH->worldOrigin, invCell);
    if (m_check_errors) {
        gpuCheckError();
    }

    thrust::sort_by_key(keys.begin(), keys.end(), m_data_mgr.activityOrderD.begin());
}

void SphFluidDynamics::UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD, double time, bool force_full) {
    const auto& countersH = m_data_mgr.countersH;
    const auto& paramsH = m_data_mgr.paramsH;

    uint numAllMarkers = (uint)countersH->numAllMarkers;
    size_t numTotal = countersH->numFsiBodies + countersH->numFsiNodes1D + countersH->numFsiNodes2D;

    // The chunk shortcut only has anything to skip once markers can actually go inactive.
    bool chunked = (numTotal > 0) && (time >= paramsH->settlingTime);

    if (!chunked) {
        m_activity_chunks_valid = false;

        uint numBlocks, numThreads;
        computeGridSize(numAllMarkers, 1024, numBlocks, numThreads);
        UpdateActivityD<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
                                                   mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->pos),
                                                   INT_32CAST(m_data_mgr.activityIdentifierOriginalD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
                                                   mR4CAST(sphMarkersD->rhoPresMuD), time);
        if (m_check_errors) {
            gpuCheckError();
        }
        return;
    }

    // Bounding box of the union of the per-solid extended active domains. Conservative with respect
    // to the per-solid test done inside the kernel: a chunk rejected against this box is outside
    // every solid's domain. The solid states live on the host as well (the device copies are always
    // filled from them), so this costs nothing on the device.
    Real3 ExAcdomain = paramsH->bodyActiveDomain + mR3(2 * paramsH->h_multiplier * paramsH->h);
    Real3 regionMin = mR3(Real(3.4e38));
    Real3 regionMax = mR3(Real(-3.4e38));
    auto accum = [&](const thrust::host_vector<Real3>& pts, size_t n) {
        for (size_t i = 0; i < n; i++) {
            const Real3& p = pts[i];
            regionMin = mR3(fmin(regionMin.x, p.x), fmin(regionMin.y, p.y), fmin(regionMin.z, p.z));
            regionMax = mR3(fmax(regionMax.x, p.x), fmax(regionMax.y, p.y), fmax(regionMax.z, p.z));
        }
    };
    accum(m_data_mgr.fsiBodyState_H->pos, countersH->numFsiBodies);
    accum(m_data_mgr.fsiMesh1DState_H->pos, countersH->numFsiNodes1D);
    accum(m_data_mgr.fsiMesh2DState_H->pos, countersH->numFsiNodes2D);
    regionMin = regionMin - ExAcdomain;
    regionMax = regionMax + ExAcdomain;

    // If the active region already spans the whole computational domain - which is the case when the
    // active-domain feature is simply not in use (bodyActiveDomain defaults to 1e10) - no chunk can
    // ever be dormant. Fall back to the flat kernel rather than pay for the per-chunk bookkeeping and
    // the indirection through activityOrderD on every marker, every step.
    Real3 worldMin = paramsH->worldOrigin;
    Real3 worldMax = paramsH->worldOrigin + paramsH->boxDims;
    if (regionMin.x <= worldMin.x && regionMin.y <= worldMin.y && regionMin.z <= worldMin.z &&  //
        regionMax.x >= worldMax.x && regionMax.y >= worldMax.y && regionMax.z >= worldMax.z) {
        m_activity_chunks_valid = false;

        uint numBlocks, numThreads;
        computeGridSize(numAllMarkers, 1024, numBlocks, numThreads);
        UpdateActivityD<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
                                                   mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->pos),
                                                   INT_32CAST(m_data_mgr.activityIdentifierOriginalD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
                                                   mR4CAST(sphMarkersD->rhoPresMuD), time);
        if (m_check_errors) {
            gpuCheckError();
        }
        return;
    }

    uint numChunks = (numAllMarkers + ACTIVITY_CHUNK_SIZE - 1) / ACTIVITY_CHUNK_SIZE;
    if (m_data_mgr.activityChunkDormant.size() != numChunks) {
        m_data_mgr.activityChunkDormant.resize(numChunks);
        m_data_mgr.activityChunkAabbMin.resize(numChunks);
        m_data_mgr.activityChunkAabbMax.resize(numChunks);
        m_activity_chunks_valid = false;
    }

    if (!m_activity_chunks_valid || force_full) {
        // Either the cache has never been filled or markers have moved behind the solver's back.
        // Rebuild the spatial ordering and visit every chunk to refill the boxes.
        BuildActivityOrder(sphMarkersD);
        force_full = true;
        m_activity_chunks_valid = true;
    }

    UpdateActivityChunkedD<<<numChunks, ACTIVITY_BLOCK_SIZE>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR3CAST(m_data_mgr.fsiMesh1DState_D->pos),
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos), INT_32CAST(m_data_mgr.activityIdentifierOriginalD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
        time, regionMin, regionMax, mR3CAST(m_data_mgr.activityChunkAabbMin), mR3CAST(m_data_mgr.activityChunkAabbMax), INT_32CAST(m_data_mgr.activityChunkDormant),
        U1CAST(m_data_mgr.activityOrderD), force_full ? 1 : 0);
    if (m_check_errors) {
        gpuCheckError();
    }
}

// -----------------------------------------------------------------------------

// Resize data based on the active particles
// Custom functor for exclusive scan that treats -1 (zombie particles) the same as 0 (sleep particles)
// Marker is in the extended active set. Zombie markers carry -1 and, like sleeping ones, do not count.
struct is_extended_active {
    __host__ __device__ bool operator()(const int32_t& v) const { return v > 0; }
};

bool SphFluidDynamics::CheckActivityArrayResize() {
    auto& countersH = m_data_mgr.countersH;

    // Build the active list directly by stream compaction. This used to be an exclusive scan over the
    // per-marker flags followed by a separate scatter kernel (fillActiveListD) - two full passes over
    // an array sized for the whole world, of which the scan also wrote a second world-sized array.
    // copy_if is stable, so the list comes out in ascending marker index, exactly as before.
    auto end = thrust::copy_if(thrust::device,                                                                //
                               thrust::counting_iterator<uint>(0),                                            //
                               thrust::counting_iterator<uint>((uint)countersH->numAllMarkers),                //
                               m_data_mgr.extendedActivityIdentifierOriginalD.begin(),                        // stencil
                               m_data_mgr.activeListD.begin(),                                                //
                               is_extended_active());

    countersH->numExtendedParticles = (size_t)(end - m_data_mgr.activeListD.begin());

    return countersH->numExtendedParticles < countersH->numAllMarkers;
}

// -----------------------------------------------------------------------------

__device__ void PositionEulerStep(Real dT, const Real3& vel, Real4& pos) {
    Real3 p = mR3(pos);
    p += dT * vel;
    pos = mR4(p, pos.w);
}

__device__ void PositionMidpointStep(Real dT, const Real3& vel, const Real3& acc, Real4& pos) {
    Real3 p = mR3(pos);
    p += dT * vel + 0.5 * dT * dT * acc;
    pos = mR4(p, pos.w);
}

__device__ void VelocityEulerStep(Real dT, const Real3& acc, Real3& vel) {
    vel += dT * acc;
}

__device__ void DensityEulerStep(Real dT, const Real& deriv, EosType eos, Real4& rho_p) {
    rho_p.x += dT * deriv;
    rho_p.y = Eos(rho_p.x, eos);
}

__device__ void TauEulerStep(Real dT,
                             const Real3& deriv_tau_diag,
                             const Real3& deriv_tau_offdiag,
                             const Real& deriv_rho,
                             bool close_to_surface,
                             Real3& tau_diag,
                             Real3& tau_offdiag,
                             Real4& rho_p,
                             Real3& pcEvSv,
                             volatile bool* error_flag) {
    if (paramsD.rheology_model_crm == RheologyCRM::MU_OF_I) {
        Real3 new_tau_diag = tau_diag + dT * deriv_tau_diag;
        Real3 new_tau_offdiag = tau_offdiag + dT * deriv_tau_offdiag;

        // Check for plastic flow
        Real p_n = -CH_1_3 * (tau_diag.x + tau_diag.y + tau_diag.z);
        Real p_tr = -CH_1_3 * (new_tau_diag.x + new_tau_diag.y + new_tau_diag.z);
        // Tau now becomes the deviatoric component
        // reusing the same register so names get confusing
        tau_diag += mR3(p_n);
        new_tau_diag += mR3(p_tr);

        Real tau_n = square(tau_diag.x) + square(tau_diag.y) + square(tau_diag.z) +                             //
                     2 * (square(tau_offdiag.x) + square(tau_offdiag.y) + square(tau_offdiag.z));               //
        Real tau_tr = square(new_tau_diag.x) + square(new_tau_diag.y) + square(new_tau_diag.z) +                //
                      2 * (square(new_tau_offdiag.x) + square(new_tau_offdiag.y) + square(new_tau_offdiag.z));  //
        tau_n = sqrt(0.5 * tau_n);
        tau_tr = sqrt(0.5 * tau_tr);
        Real Chi = abs(tau_tr - tau_n) * paramsD.INV_G_shear / dT;

        // Should use the positive magnitude according to "A constitutive law for dense granular flows" Nature 2006
        Real mu_s = paramsD.mu_fric_s;
        Real mu_2 = paramsD.mu_fric_2;
        // Real s_0 = mu_s * p_tr;
        // Real s_2 = mu_2 * p_tr;
        // Real xi = 1.1;
        Real dia = paramsD.ave_diam;
        Real I0 = paramsD.mu_I0;  // xi*dia*sqrt(rhoPresMu.x);//

        // Zero-tension cutoff (Mohr-Coulomb tension cut-off convention): cohesion
        // contributes shear strength through tau_max = mu * p + c below, but grants
        // no tension capacity. The previous cutoff at p_tr < -c/mu_s let particles
        // sustain negative pressure, which (a) drives the SPH tensile instability
        // (particle clumping that destabilizes geostatic stress states and collapses
        // bearing responses whenever c > 0), and (b) made the inertial number I NaN
        // for p_tr < 0 (sqrt of a negative), silently disabling the yield check for
        // tensile particles. For c = 0 this update is identical to the previous one.
        if (p_tr < Real(0))
            p_tr = Real(0);

        Real I = Chi * dia * sqrt(paramsD.rho0 / (p_tr + 1.0e-9));

        Real coh = paramsD.Coh_coeff;
        // Real Chi_cri = 0.1;
        // if (Chi < Chi_cri){
        //     coh = paramsD.Coh_coeff * (1.0 - sin(-1.57 + 3.14 * (Chi / Chi_cri))) / 2.0;
        //     // coh = paramsD.Coh_coeff * (1.0 - I / I_cri);
        // } else {
        //     coh = 0.0;
        // }

        Real mu = mu_s + (mu_2 - mu_s) * (I + 1.0e-9) / (I0 + I + 1.0e-9);
        // Real G0 = paramsD.G_shear;
        // Real alpha = xi*G0*I0*(dT)*sqrt(p_tr);
        // Real B0 = s_2 + tau_tr + alpha;
        // Real H0 = s_2*tau_tr + s_0*alpha;
        // Real tau_n1 = (B0+sqrt(B0*B0-4*H0))/(2*H0+1e-9);
        // if(tau_tr>s_0){
        //     Real coeff = tau_n1/(tau_tr+1e-9);
        //     updatedTauXxYyZz = updatedTauXxYyZz*coeff;
        //     updatedTauXyXzYz = updatedTauXyXzYz*coeff;
        // }
        Real tau_max = p_tr * mu + coh;  // p_tr*paramsD.Q_FA;
        // should use tau_max instead of s_0 according to
        // "A constitutive law for dense granular flows" Nature 2006
        if (tau_tr > tau_max) {
            Real coeff = tau_max / (tau_tr + 1e-9);
            new_tau_diag *= coeff;
            new_tau_offdiag *= coeff;
        }

        // Set stress to zero if the particle is close to free surface
        if (close_to_surface == 1) {
            new_tau_diag = mR3(0.0);
            new_tau_offdiag = mR3(0.0);
            p_tr = 0.0;
        }
        // Going back to sigma from the deviatoric component
        tau_diag = new_tau_diag - mR3(p_tr);
        tau_offdiag = new_tau_offdiag;

        rho_p.y = p_tr;
        // rho_p.x = rho_p.x + deriv_rho * dT;
        rho_p.x = paramsD.rho0;
    } else {
        // Implementation reference
        // https://docs.itascacg.com/flac3d700/common/models/camclay/doc/modelcamclay.html#equation-modelmcceqvn
        // N is state at next time step
        // n is state at current time step
        // N_tr is trial
        Real mcc_M = paramsD.mcc_M;
        Real mcc_lambda = paramsD.mcc_lambda;
        Real mcc_kappa = paramsD.mcc_kappa;
        Real p_c = pcEvSv.x;
        // Use the previous time steps specific volume
        Real specific_volume_n = pcEvSv.z;

        // Compute local bulk/shear moduli per MCC (Itasca Eq. (15), (46))
        // Use previous-state mean pressure with floors, clamp and under-relax for stability
        // Recomputing for now instead of storing for each particle to avoid memory usage (compute once in crmRHS)
        Real p_n = -CH_1_3 * (tau_diag.x + tau_diag.y + tau_diag.z);
        // Candidate bulk modulus from MCC
        Real K_cand = specific_volume_n * (p_n) / paramsD.mcc_kappa;
        // Clamp K
        Real K_n = fmin(fmax(K_cand, Real(0.1) * paramsD.K_bulk), Real(1.0) * paramsD.K_bulk);
        // Shear
        Real G_cand = (3.0 * K_n * (1.0 - 2.0 * paramsD.Nu_poisson)) / (2.0 * (1.0 + paramsD.Nu_poisson));
        Real G_n = fmin(fmax(G_cand, Real(0.1) * paramsD.G_shear), Real(1.0) * paramsD.G_shear);
        // Trial stress using convention N = n + 1
        Real3 sig_diag_N_tr = tau_diag + dT * deriv_tau_diag;
        Real3 sig_offdiag_N_tr = tau_offdiag + dT * deriv_tau_offdiag;
        Real p_N_tr = -CH_1_3 * (sig_diag_N_tr.x + sig_diag_N_tr.y + sig_diag_N_tr.z);
        // Deviatoric component of the trial stress
        Real3 dev_diag_N_tr = sig_diag_N_tr + mR3(p_N_tr);
        Real3 dev_offdiag_N_tr = sig_offdiag_N_tr;

        // Computing trial von misses stress (q_N_tr)
        Real inner_product = square(dev_diag_N_tr.x) + square(dev_diag_N_tr.y) + square(dev_diag_N_tr.z) +
                             2 * (square(dev_offdiag_N_tr.x) + square(dev_offdiag_N_tr.y) + square(dev_offdiag_N_tr.z));
        Real J_2 = inner_product * 0.5;
        Real q_N_tr = sqrt(3.0 * J_2);

        // Computing yield function (f_N)
        Real f_N = square(q_N_tr) + square(mcc_M) * p_N_tr * (p_N_tr - p_c);
        Real3 s_diag_N = sig_diag_N_tr;
        Real3 s_offdiag_N = sig_offdiag_N_tr;
        Real p_N = p_N_tr;
        Real delta_lambda_N = 0.0;
        Real c_v = 0.0;
        // Scale-aware tolerances
        Real f_scale = square(q_N_tr) + square(mcc_M) * square(p_N_tr);
        Real f_tol = fmax(Real(1e-12), Real(1e-6) * f_scale);
        Real q_eps = fmax(Real(1e-9), Real(1e-6) * (fabs(p_N_tr) + q_N_tr));

        // No tension
        if (p_N_tr < 0) {
            tau_diag = mR3(0.0);
            tau_offdiag = mR3(0.0);
            rho_p.y = 0.0;
        } else if (p_N_tr > 0 && f_N <= f_tol) {
            // Nearly on the yield surface: treat as elastic
            tau_diag = sig_diag_N_tr;
            tau_offdiag = sig_offdiag_N_tr;
            rho_p.y = p_N_tr;
        } else if (p_N_tr > 0 && f_N > 0) {  // If we have yielded, find the new deviatoric stress
            c_v = square(mcc_M) * (2 * p_N_tr - p_c);
            Real c_q = 2 * q_N_tr;
            // Quadratic coefficients for plastic strain increment (delta lambda)
            Real a = square(mcc_M * K_n * c_v) + square(3 * G_n * c_q);
            Real b = -K_n * square(c_v) - 3 * G_n * square(c_q);
            Real c = f_N;
            // q -> 0 guard: pure volumetric correction
            if (q_N_tr < q_eps) {
                c_q = 0.0;
                a = square(mcc_M * K_n * c_v);
                b = -K_n * square(c_v);
            }
            // Solve quadratic robustly, in double precision: in single precision b^2
            // overflows to inf once |b| > 1.84e19, which is reached for q_N_tr ~ 390 kPa
            // at clamped moduli (b ~ -3*G*(2q)^2). The inf then propagates through
            // delta_lambda to the hardening update and permanently poisons p_c.
            if (a <= 0) {
                delta_lambda_N = 0.0;
            } else {
                double ad = (double)a;
                double bd = (double)b;
                double cd = (double)c;
                double disc = fmax(bd * bd - 4.0 * ad * cd, 0.0);
                double sqrt_disc = sqrt(disc);
                double inv_2a = 0.5 / ad;
                double r1 = (-bd + sqrt_disc) * inv_2a;
                double r2 = (-bd - sqrt_disc) * inv_2a;
                // pick the smallest positive root (or 0 if none)
                double dl = 0.0;
                if (r1 > 0 && r2 > 0)
                    dl = (r1 < r2) ? r1 : r2;
                else if (r1 > 0)
                    dl = r1;
                else if (r2 > 0)
                    dl = r2;
                delta_lambda_N = (Real)dl;
            }

            // Get the mapped stress
            p_N = p_N_tr - K_n * delta_lambda_N * c_v;
            Real q_N = (q_N_tr - 3 * G_n * delta_lambda_N * c_q);
            s_diag_N = q_N * dev_diag_N_tr / (q_N_tr + q_eps);
            s_offdiag_N = q_N * dev_offdiag_N_tr / (q_N_tr + q_eps);

            // No tension allowed, else get the new stress from the mapped deviatoric stress and pressure
            if (p_N < 0) {
                tau_diag = mR3(0.0);
                tau_offdiag = mR3(0.0);
                rho_p.y = 0.0;
            } else {
                tau_diag = s_diag_N - mR3(p_N);
                tau_offdiag = s_offdiag_N;
                rho_p.y = p_N;
            }
            // Update the consolidation pressure (only if we are not close to the free surface).
            // Guard against a non-finite plastic strain increment: without this, one bad
            // delta_lambda makes p_c infinite forever (the fmax floor does not catch inf)
            // and the particle never yields again.
            Real plastic_volumentric_strain = delta_lambda_N * c_v;
            if (!close_to_surface && isfinite(plastic_volumentric_strain)) {
                pcEvSv.x *= (1 + plastic_volumentric_strain * (specific_volume_n / (mcc_lambda - mcc_kappa)));
                // pcEvSv.x *= exp(plastic_volumentric_strain * (specific_volume_n / (mcc_lambda - mcc_kappa)));
                pcEvSv.x = fmax(Real(100.0), pcEvSv.x);
            }
        }

        // If we are close to free surface, set stress tensor to zero tensor
        // TODO: Should the consolidation pressure also be zero? This makes the material have zero yield
        if (close_to_surface == 1) {
            tau_diag = mR3(0.0);
            tau_offdiag = mR3(0.0);
            rho_p.y = 0.0;
        }
        // Update density
        rho_p.x = rho_p.x + deriv_rho * dT;
        // Update specific volume based on the volumetric strain rate
        // TODO: How are we guaranteed that the volumetric strain rate and the density are synchronized?
        // One aspect is that they are both numerically integrated from the divergence of the velocity field
        pcEvSv.z *= (1 - pcEvSv.y * dT);
        // Set min to prevent collapse of the specific volume
        pcEvSv.z = fmax(Real(1.0), pcEvSv.z);
    }

    // Flag a rheology failure (non-finite updated stress state) so the host can abort.
    // This complements the host-side position/density NaN scans, which do not cover stress.
    // Nothing is evaluated when error checking is disabled (error_flag is then null).
    // Check only state the active rheology model integrates: mu(I) neither reads nor
    // updates the consolidation state (pcEvSv), which can legitimately be non-finite
    // straight out of default initialization (AddSphParticle derives the specific volume
    // with log(pc / p1), which is not finite for the default zero initial pressure).
    if (error_flag) {
        bool rheology_failed = !IsFinite(tau_diag) || !IsFinite(tau_offdiag) || !IsFinite(rho_p);
        if (paramsD.rheology_model_crm == RheologyCRM::MCC)
            rheology_failed = rheology_failed || !IsFinite(pcEvSv);
        if (rheology_failed)
            *error_flag = true;
    }
}

// Kernel to update the fluid properties of a particle, using an explicit Euler step.
// First, update the particle position and velocity. Next,
// - For a CFD problem, advance the density and calculate pressure from the Equation of State;
// - For a CRM problem, update the stress tensor and the pressure (density is kept constant).
//
// Important note: the derivVelRhoD calculated by ChForceExplicitSPH is the negative of actual time
// derivative. That is important to keep the derivVelRhoD to be the force/mass for fsi forces.
// - calculate the force, that is f=m dv/dt
// - derivVelRhoD[index] *= paramsD.markerMass;
__global__ void EulerStep_D(Real4* posRadD,
                            Real3* velMasD,
                            Real4* rhoPresMuD,
                            Real3* tauXxYyZzD,
                            Real3* tauXyXzYzD,
                            Real3* pcEvSvD,
                            const Real3* vel_XSPH_D,
                            const Real4* derivVelRhoD,
                            const Real3* derivTauXxYyZzD,
                            const Real3* derivTauXyXzYzD,
                            const uint* freeSurfaceIdD,
                            const int32_t* activityIdentifierSortedD,
                            const uint numActive,
                            Real dT,
                            volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    // Only update active SPH particles, not extended active particles
    if (IsBceMarker(rhoPresMuD[index].w) || activityIdentifierSortedD[index] <= 0)
        return;

    // Euler step for position
    PositionEulerStep(dT, velMasD[index] + vel_XSPH_D[index], posRadD[index]);

    // Euler step for velocity
    VelocityEulerStep(dT, mR3(derivVelRhoD[index]), velMasD[index]);

    if (paramsD.elastic_SPH) {
        // Euler step for tau and pressure update
        TauEulerStep(dT, derivTauXxYyZzD[index], derivTauXyXzYzD[index], derivVelRhoD[index].w, freeSurfaceIdD[index], tauXxYyZzD[index], tauXyXzYzD[index], rhoPresMuD[index],
                     pcEvSvD[index], error_flag);
    } else {
        // Euler step for density and pressure update from EOS
        DensityEulerStep(dT, derivVelRhoD[index].w, paramsD.eos_type, rhoPresMuD[index]);
    }
}

// Kernel to update the fluid properties of a particle, using an mid-point step.
// Note: the derivatives (provided in input vectors) are assumed to have been calculated at the mid-point!
// The mid-point updates for position and velocity are:
//    v_{n+1} = v_n + h * F_{n+1/2}
//    r_{n+1} = r_n + h * (v_{n+1} + v_n) / 2
// These are implemented in reverse order (because the velocity update would overwrite v_n) as:
//    r_{n+1} = r_n + h * v_n + 0.5 * h^2 * F_{n+1/2}
//    v_{n+1} = v_n + h * F_{n+1/2}
// After the position and velocity updates, the mid-point update for density (CFD) or stress (CRM) are equivalent to the
// velocity update above (i.e., an Euler step).
__global__ void MidpointStep_D(Real4* posRadD,
                               Real3* velMasD,
                               Real4* rhoPresMuD,
                               Real3* tauXxYyZzD,
                               Real3* tauXyXzYzD,
                               Real3* pcEvSvD,
                               const Real3* vel_XSPH_D,
                               const Real4* derivVelRhoD,
                               const Real3* derivTauXxYyZzD,
                               const Real3* derivTauXyXzYzD,
                               const uint* freeSurfaceIdD,
                               const int32_t* activityIdentifierSortedD,
                               const uint numActive,
                               Real dT,
                               volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    // Only update active SPH particles, not extended active particles
    if (IsBceMarker(rhoPresMuD[index].w) || activityIdentifierSortedD[index] <= 0)
        return;

    // Advance position
    //// TODO: what about XSPH?
    PositionMidpointStep(dT, velMasD[index] + vel_XSPH_D[index], mR3(derivVelRhoD[index]), posRadD[index]);

    // Advance velocity
    VelocityEulerStep(dT, mR3(derivVelRhoD[index]), velMasD[index]);

    if (paramsD.elastic_SPH) {
        // Euler step for tau and pressure update
        TauEulerStep(dT, derivTauXxYyZzD[index], derivTauXyXzYzD[index], derivVelRhoD[index].w, freeSurfaceIdD[index], tauXxYyZzD[index], tauXyXzYzD[index], rhoPresMuD[index],
                     pcEvSvD[index], error_flag);
    } else {
        // Euler step for density and pressure update from EOS
        DensityEulerStep(dT, derivVelRhoD[index].w, paramsD.eos_type, rhoPresMuD[index]);
    }
}

template <typename T>
struct check_infinite {
    __host__ __device__ bool operator()(const T& v) { return !IsFinite(v); }
};

void SphFluidDynamics::EulerStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);

    bool* error_flagD = nullptr;
    if (m_check_errors) {
        gpuResetErrorFlag(m_errflagD);
        error_flagD = m_errflagD;
    }

    EulerStep_D<<<numBlocks, numThreads>>>(mR4CAST(sortedMarkers->posRadD), mR3CAST(sortedMarkers->velMasD), mR4CAST(sortedMarkers->rhoPresMuD), mR3CAST(sortedMarkers->tauXxYyZzD),
                                           mR3CAST(sortedMarkers->tauXyXzYzD), mR3CAST(sortedMarkers->pcEvSvD), mR3CAST(m_data_mgr.vel_XSPH_D), mR4CAST(m_data_mgr.derivVelRhoD),
                                           mR3CAST(m_data_mgr.derivTauXxYyZzD), mR3CAST(m_data_mgr.derivTauXyXzYzD), U1CAST(m_data_mgr.freeSurfaceIdD),
                                           INT_32CAST(m_data_mgr.activityIdentifierSortedD), numActive, dT, error_flagD);

    if (m_check_errors) {
        gpuCheckError();
        if (thrust::any_of(sortedMarkers->posRadD.begin(), sortedMarkers->posRadD.begin() + numActive, check_infinite<Real4>()))
            gpuThrowError("A particle position is NaN");
        if (thrust::any_of(sortedMarkers->rhoPresMuD.begin(), sortedMarkers->rhoPresMuD.begin() + numActive, check_infinite<Real4>()))
            gpuThrowError("A particle density is NaN");
        // Even if one particle has this problem, we can't proceed
        gpuCheckErrorFlag(error_flagD, "TauEulerStep (rheology model failure)");
    }
}

void SphFluidDynamics::MidpointStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);

    bool* error_flagD = nullptr;
    if (m_check_errors) {
        gpuResetErrorFlag(m_errflagD);
        error_flagD = m_errflagD;
    }
    MidpointStep_D<<<numBlocks, numThreads>>>(
        mR4CAST(sortedMarkers->posRadD), mR3CAST(sortedMarkers->velMasD), mR4CAST(sortedMarkers->rhoPresMuD), mR3CAST(sortedMarkers->tauXxYyZzD),
        mR3CAST(sortedMarkers->tauXyXzYzD), mR3CAST(sortedMarkers->pcEvSvD), mR3CAST(m_data_mgr.vel_XSPH_D), mR4CAST(m_data_mgr.derivVelRhoD), mR3CAST(m_data_mgr.derivTauXxYyZzD),
        mR3CAST(m_data_mgr.derivTauXyXzYzD), U1CAST(m_data_mgr.freeSurfaceIdD), INT_32CAST(m_data_mgr.activityIdentifierSortedD), numActive, dT, error_flagD);

    if (m_check_errors) {
        gpuCheckError();
        if (thrust::any_of(sortedMarkers->posRadD.begin(), sortedMarkers->posRadD.begin() + numActive, check_infinite<Real4>()))
            gpuThrowError("A particle position is NaN");
        if (thrust::any_of(sortedMarkers->rhoPresMuD.begin(), sortedMarkers->rhoPresMuD.begin() + numActive, check_infinite<Real4>()))
            gpuThrowError("A particle density is NaN");
        // Even if one particle has this problem, we can't proceed
        gpuCheckErrorFlag(error_flagD, "TauEulerStep (rheology model failure)");
    }
}

// -----------------------------------------------------------------------------

// Kernel to copy sorted data back to original order (ISPH)
__global__ void CopySortedToOriginalISPH_D(MarkerGroup group,
                                           const Real4* sortedPosRad,
                                           const Real3* sortedVelMas,
                                           const Real4* sortedRhoPresMu,
                                           const Real3* sortedTauXxYyZz,
                                           const Real3* sortedTauXyXXzYz,
                                           const Real4* derivVelRho,
                                           const Real4* sr_tau_I_mu_i,
                                           const uint numActive,
                                           Real4* posRadOriginal,
                                           Real3* velMasOriginal,
                                           Real4* rhoPresMuOriginal,
                                           Real3* tauXxYyZzOriginal,
                                           Real3* tauXyXzYzOriginal,
                                           Real4* derivVelRhoOriginal,
                                           Real4* sr_tau_I_mu_i_Original,
                                           uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numActive)
        return;

    Real type = sortedRhoPresMu[id].w;
    if (!IsInMarkerGroup(group, type))
        return;

    uint index = gridMarkerIndex[id];
    posRadOriginal[index] = sortedPosRad[id];
    velMasOriginal[index] = sortedVelMas[id];
    rhoPresMuOriginal[index] = sortedRhoPresMu[id];
    derivVelRhoOriginal[index] = derivVelRho[id];
    tauXxYyZzOriginal[index] = sortedTauXxYyZz[id];
    tauXyXzYzOriginal[index] = sortedTauXyXXzYz[id];
    sr_tau_I_mu_i_Original[index] = sr_tau_I_mu_i[id];
}

// Kernel to copy sorted data back to original order (WCSPH)
__global__ void CopySortedToOriginalWCSPH_D(MarkerGroup group,
                                            const Real4* sortedPosRad,
                                            const Real3* sortedVelMas,
                                            const Real4* sortedRhoPresMu,
                                            const Real3* sortedTauXxYyZz,
                                            const Real3* sortedTauXyXXzYz,
                                            const Real3* sortedPcEvSv,
                                            const Real4* derivVelRho,
                                            const uint numActive,
                                            Real4* posRadOriginal,
                                            Real3* velMasOriginal,
                                            Real4* rhoPresMuOriginal,
                                            Real3* tauXxYyZzOriginal,
                                            Real3* tauXyXzYzOriginal,
                                            Real3* pcEvSvOriginal,
                                            Real4* derivVelRhoOriginal,
                                            uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numActive)
        return;

    Real type = sortedRhoPresMu[id].w;
    if (!IsInMarkerGroup(group, type))
        return;

    uint index = gridMarkerIndex[id];
    posRadOriginal[index] = sortedPosRad[id];
    velMasOriginal[index] = sortedVelMas[id];
    rhoPresMuOriginal[index] = sortedRhoPresMu[id];
    derivVelRhoOriginal[index] = derivVelRho[id];
    tauXxYyZzOriginal[index] = sortedTauXxYyZz[id];
    tauXyXzYzOriginal[index] = sortedTauXyXXzYz[id];
    pcEvSvOriginal[index] = sortedPcEvSv[id];
}

void SphFluidDynamics::CopySortedToOriginal(MarkerGroup group, std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);
    if (m_data_mgr.paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH) {
        CopySortedToOriginalISPH_D<<<numBlocks, numThreads>>>(
            group, mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD), mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(m_data_mgr.derivVelRhoD), mR4CAST(m_data_mgr.sr_tau_I_mu_i), numActive, mR4CAST(sphMarkersD->posRadD),
            mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD), mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD),
            mR4CAST(m_data_mgr.derivVelRhoOriginalD), mR4CAST(m_data_mgr.sr_tau_I_mu_i_Original), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
    } else {
        CopySortedToOriginalWCSPH_D<<<numBlocks, numThreads, 0, m_copy_stream>>>(
            group, mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD), mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR3CAST(sortedSphMarkersD->pcEvSvD), mR4CAST(m_data_mgr.derivVelRhoD), numActive, mR4CAST(sphMarkersD->posRadD),
            mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD), mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD), mR3CAST(sphMarkersD->pcEvSvD),
            mR4CAST(m_data_mgr.derivVelRhoOriginalD), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
    }
    if (m_check_errors) {
        gpuCheckError();
    }
}

// -----------------------------------------------------------------------------

// Kernel to apply inlet/outlet BC along x
__global__ void ApplyInletBoundaryX_D(Real4* posRadD, Real3* VelMassD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.x;
        rhoPresMuD[index] = rhoPresMu;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.x;
        rhoPresMuD[index] = rhoPresMu;
    }

    if (posRad.x > -paramsD.x_in)
        rhoPresMuD[index].y = 0;

    if (posRad.x < paramsD.x_in)
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
}

// Kernel to apply periodic BC along x
__global__ void ApplyPeriodicBoundaryX_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMuD[index].y += paramsD.delta_pressure.x;
        return;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMuD[index].y -= paramsD.delta_pressure.x;
        return;
    }
}

// Kernel to apply periodic BC along y
__global__ void ApplyPeriodicBoundaryY_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.y > paramsD.cMax.y) {
        posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.y;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
    if (posRad.y < paramsD.cMin.y) {
        posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.y;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
}

// Kernel to apply periodic BC along z
__global__ void ApplyPeriodicBoundaryZ_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.z > paramsD.cMax.z) {
        posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.z;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
    if (posRad.z < paramsD.cMin.z) {
        posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.z;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
}

// Apply boundary conditions in x, y, and z directions
void SphFluidDynamics::ApplyBoundaryConditions(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);

    switch (m_data_mgr.paramsH->bc_type.x) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryX_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            if (m_check_errors) {
                gpuCheckError();
            }
            break;
        case BCType::INLET_OUTLET:
            //// TODO - check this and modify as appropriate
            // ApplyInletBoundaryX_D<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
            //                                                  mR3CAST(sphMarkersD->velMasD),
            //                                                  mR4CAST(sphMarkersD->rhoPresMuD), numActive);
            // gpuCheckError();
            break;
    }

    switch (m_data_mgr.paramsH->bc_type.y) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryY_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            if (m_check_errors) {
                gpuCheckError();
            }
            break;
    }

    switch (m_data_mgr.paramsH->bc_type.z) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryZ_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            if (m_check_errors) {
                gpuCheckError();
            }
            break;
    }
}

// -----------------------------------------------------------------------------

// Device function to calculate the share of density influence on a given
// particle from all other particle in a given cell
__device__ void collideCellDensityReInit(Real& numerator,
                                         Real& denominator,
                                         int3 gridPos,
                                         uint index,
                                         Real3 posRadA,
                                         Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         uint* cellStart,
                                         uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    uint startIndex = cellStart[gridHash];
    if (startIndex != 0xffffffff) {  // cell is not empty
        // iterate over particles in this cell
        uint endIndex = cellEnd[gridHash];
        for (uint j = startIndex; j < endIndex; j++) {
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real4 rhoPreMuB = sortedRhoPreMu[j];
            Real3 dist3 = Distance(posRadA, posRadB);
            Real d = length(dist3);
            if (d > paramsD.h_multiplier * paramsD.h)
                continue;
            Real w = W3h(paramsD.kernel_type, d, paramsD.ooh);
            numerator += paramsD.markerMass * w;
            denominator += paramsD.markerMass / rhoPreMuB.x * w;
        }
    }
}

// Kernel for updating the density.
// It calculates the density of the particle. It does include the normalization
// close to the boundaries and free surface.
__global__ void
ReCalcDensityD_F1(Real4* dummySortedRhoPreMu, Real4* sortedPosRad, Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* gridMarkerIndex, uint* cellStart, uint* cellEnd) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    // read particle data from sorted arrays
    Real3 posRadA = mR3(sortedPosRad[index]);
    Real4 rhoPreMuA = sortedRhoPreMu[index];

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    Real numerator = 0.0;
    Real denominator = 0.0;
    // examine neighboring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                collideCellDensityReInit(numerator, denominator, neighbourPos, index, posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }

    rhoPreMuA.x = numerator;  // denominator;
    //    rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
    dummySortedRhoPreMu[index] = rhoPreMuA;
}

void SphFluidDynamics::DensityReinitialization() {
    uint numBlocks, numThreads;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 256, numBlocks, numThreads);

    thrust::device_vector<Real4> dummySortedRhoPreMu(m_data_mgr.countersH->numAllMarkers);
    thrust::fill(dummySortedRhoPreMu.begin(), dummySortedRhoPreMu.end(), mR4(0.0));

    ReCalcDensityD_F1<<<numBlocks, numThreads>>>(mR4CAST(dummySortedRhoPreMu), mR4CAST(m_data_mgr.sortedSphMarkers1_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers1_D->velMasD),
                                                 mR4CAST(m_data_mgr.sortedSphMarkers1_D->rhoPresMuD), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
                                                 U1CAST(m_data_mgr.markersProximity_D->cellStartD), U1CAST(m_data_mgr.markersProximity_D->cellEndD));
    if (m_check_errors) {
        gpuCheckError();
    }
    SphForce::CopySortedToOriginal_NonInvasive_R4(m_data_mgr.sphMarkers_D->rhoPresMuD, dummySortedRhoPreMu, m_data_mgr.markersProximity_D->gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace sph
}  // namespace fsi
}  // end namespace chrono
