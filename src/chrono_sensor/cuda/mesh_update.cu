// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Device-side helpers for incremental deformable mesh updates.
//
// =============================================================================

#include <cuda.h>
#include <cuda_runtime.h>

#include "mesh_update.cuh"

namespace chrono {
namespace sensor {

__global__ void scatter_float4_kernel(const int* __restrict__ indices,
                                      const float4* __restrict__ values,
                                      float4* __restrict__ dest,
                                      unsigned int count,
                                      unsigned int dest_count) {
    unsigned int k = blockDim.x * blockIdx.x + threadIdx.x;
    if (k >= count)
        return;
    int idx = indices[k];
    // Rejected rather than clamped: clamping would quietly corrupt a real vertex, whereas dropping
    // leaves the stale value in place, which is both visible and harmless.
    if (idx < 0 || (unsigned int)idx >= dest_count)
        return;
    dest[idx] = values[k];
}

void cuda_scatter_float4(const void* d_indices,
                         const void* d_values,
                         void* d_dest,
                         unsigned int count,
                         unsigned int dest_count,
                         CUstream& stream) {
    if (count == 0)
        return;
    const int nThreads = 256;
    int nBlocks = (count + nThreads - 1) / nThreads;
    scatter_float4_kernel<<<nBlocks, nThreads, 0, stream>>>((const int*)d_indices, (const float4*)d_values,
                                                            (float4*)d_dest, count, dest_count);
}

}  // namespace sensor
}  // namespace chrono
