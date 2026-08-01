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

#ifndef CHRONO_SENSOR_CUDA_MESH_UPDATE_CUH
#define CHRONO_SENSOR_CUDA_MESH_UPDATE_CUH

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// Scatter a packed array of vertex attributes into the positions named by an index array.
///
/// Used to push only the vertices a deformable mesh actually changed, instead of restating the
/// whole buffer. `values[k]` is written to `dest[indices[k]]`.
///
/// Indices at or beyond `dest_count` are skipped rather than written. The index list comes from a
/// mesh producer outside Chrono::Sensor, and a stray index here would be an out-of-bounds device
/// write into a neighbouring allocation: silent corruption that would surface far from its cause.
///
/// @param d_indices  device array of `count` destination indices
/// @param d_values   device array of `count` float4 values, parallel to d_indices
/// @param d_dest     device destination buffer holding `dest_count` float4 entries
/// @param count      number of entries to scatter
/// @param dest_count capacity of d_dest, in float4 entries, used for bounds rejection
/// @param stream     the cuda stream for the kernel
void cuda_scatter_float4(const void* d_indices,
                         const void* d_values,
                         void* d_dest,
                         unsigned int count,
                         unsigned int dest_count,
                         CUstream& stream);

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
