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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Radu Serban
// =============================================================================
//
// Utilities for GPU error testing and GPU timing
//
// =============================================================================

#ifndef CH_SPH_UTILS_DEVICE_H
#define CH_SPH_UTILS_DEVICE_H

#include <iostream>
#include <string>
#include <vector>

#include "chrono/gpu/ChGpuRuntime.h"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/execution_policy.h>

#include "chrono/core/ChTypes.h"

#include "chrono_fsi/sph/math/SphCustomMath.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_utils
/// @{

// ----------------------------------------------------------------------------
// Short-hand notation
// ----------------------------------------------------------------------------

#define mF2 make_float2
#define mF3 make_float3
#define mF4 make_float4
#define mR2 make_Real2
#define mR3 make_Real3
#define mR4 make_Real4

#define mI2 make_int2
#define mI3 make_int3
#define mI4 make_int4

#define mU3 make_uint3

#define F1CAST(x) (float*)thrust::raw_pointer_cast(&x[0])
#define D1CAST(x) (double*)thrust::raw_pointer_cast(&x[0])
#define BCAST(x) (bool*)thrust::raw_pointer_cast(&x[0])

#define I1CAST(x) (int*)thrust::raw_pointer_cast(&x[0])
// This is not 32 uints but rather a single uint32_t
#define UINT_32CAST(x) (uint32_t*)thrust::raw_pointer_cast(&x[0])
// This is not 32 ints but rather a single int32_t
#define INT_32CAST(x) (int32_t*)thrust::raw_pointer_cast(&x[0])
#define mI2CAST(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define mI3CAST(x) (int3*)thrust::raw_pointer_cast(&x[0])
#define mI4CAST(x) (int4*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define U2CAST(x) (uint2*)thrust::raw_pointer_cast(&x[0])
#define U3CAST(x) (uint3*)thrust::raw_pointer_cast(&x[0])
#define U4CAST(x) (uint4*)thrust::raw_pointer_cast(&x[0])

#define LU1CAST(x) (unsigned long int*)thrust::raw_pointer_cast(&x[0])
#define R1CAST(x) (Real*)thrust::raw_pointer_cast(&x[0])
#define mR2CAST(x) (Real2*)thrust::raw_pointer_cast(&x[0])
#define mR3CAST(x) (Real3*)thrust::raw_pointer_cast(&x[0])
#define mR4CAST(x) (Real4*)thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())
#define mR3BY3CAST(x) (Real3By3*)thrust::raw_pointer_cast(&x[0])

// Thrust execution policy for algorithms whose result is only consumed by later device work. Unlike the default
// policy, it does not synchronize the host with the device before returning.
#if defined(CHRONO_USE_HIP) && !defined(__HIP_PLATFORM_NVIDIA__)
    #define SPH_THRUST_NOSYNC thrust::hip::par_nosync
#else
    #define SPH_THRUST_NOSYNC thrust::cuda::par_nosync
#endif

// ----------------------------------------------------------------------------

// The four error-flag macros call gpuMalloc, gpuMemcpy and gpuFree, each of which returns a
// gpuError. Discarding those codes lets an allocation or copy failure pass unnoticed: the flag is
// then read from memory that was never written, and a failed device-to-host copy reports "no error"
// because error_flag_H keeps whatever the stack held. So every call is checked. gpuFreeErrorFlag
// reports rather than throws, because it runs from destructors.
#define gpuMallocErrorFlag(error_flag_D)                                    \
    {                                                                       \
        gpuError err_ = gpuMalloc((void**)&error_flag_D, sizeof(bool));     \
        if (err_ != gpuSuccess)                                             \
            gpuThrowError(gpuGetErrorString(err_));                         \
    }

#define gpuFreeErrorFlag(error_flag_D)                                                  \
    {                                                                                   \
        if (error_flag_D) {                                                             \
            bool* flag_to_free_ = error_flag_D;                                         \
            error_flag_D = nullptr; /* null first; nothing below may skip it */         \
            gpuError err_ = gpuFree(flag_to_free_);                                     \
            if (err_ != gpuSuccess) {                                                   \
                try { /* runs from destructors: report, never throw */                  \
                    std::cerr << "GPU failure in " << __FILE__ << ":" << __LINE__       \
                              << " Message: " << gpuGetErrorString(err_) << std::endl;  \
                } catch (...) {                                                         \
                }                                                                       \
            }                                                                           \
        }                                                                               \
    }

// Clear the flag on the default stream, ordered before the kernels that set it. The asynchronous memset
// does not block the host (a synchronous copy from pageable host memory would).
#define gpuResetErrorFlag(error_flag_D)                                   \
    {                                                                     \
        gpuError err_ = gpuMemsetAsync(error_flag_D, 0, sizeof(bool), 0); \
        if (err_ != gpuSuccess)                                           \
            gpuThrowError(gpuGetErrorString(err_));                       \
    }

#define gpuCheckErrorFlag(error_flag_D, kernel_name)                                                         \
    {                                                                                                        \
        bool error_flag_H = false;                                                                           \
        gpuError err_ = gpuDeviceSynchronize();                                                              \
        if (err_ != gpuSuccess)                                                                              \
            gpuThrowError(gpuGetErrorString(err_));                                                          \
        err_ = gpuMemcpy(&error_flag_H, error_flag_D, sizeof(bool), gpuMemcpyDeviceToHost);                  \
        if (err_ != gpuSuccess)                                                                              \
            gpuThrowError(gpuGetErrorString(err_));                                                          \
        if (error_flag_H) {                                                                                  \
            char buffer[256];                                                                                \
            sprintf(buffer, "Error flag intercepted in %s:%d from %s", __FILE__, __LINE__, kernel_name);     \
            std::cerr << buffer << std::endl;                                                                \
            throw std::runtime_error(buffer);                                                                \
        }                                                                                                    \
        gpuError e = gpuGetLastError(); /* a synchronous launch error is not returned by the calls above */  \
        if (e != gpuSuccess) {                                                                               \
            char buffer[256];                                                                                \
            sprintf(buffer, "GPU failure in %s:%d Message: %s", __FILE__, __LINE__, gpuGetErrorString(e));   \
            std::cerr << buffer << std::endl;                                                                \
            throw std::runtime_error(buffer);                                                                \
        }                                                                                                    \
    }

#define gpuCheckError()                                                                                    \
    {                                                                                                      \
        gpuDeviceSynchronize();                                                                            \
        gpuError e = gpuGetLastError();                                                                    \
        if (e != gpuSuccess) {                                                                             \
            char buffer[256];                                                                              \
            sprintf(buffer, "GPU failure in %s:%d Message: %s", __FILE__, __LINE__, gpuGetErrorString(e)); \
            std::cerr << buffer << std::endl;                                                              \
            throw std::runtime_error(buffer);                                                              \
        }                                                                                                  \
    }

#define gpuThrowError(message)                                                            \
    {                                                                                     \
        char buffer[256];                                                                 \
        sprintf(buffer, "GPU failure in %s:%d Message: %s", __FILE__, __LINE__, message); \
        std::cerr << buffer << std::endl;                                                 \
        throw std::runtime_error(buffer);                                                 \
    }

// Check for a kernel launch error without synchronizing with the device. Errors raised while a kernel
// executes are reported by the next synchronizing call (see GpuErrorFlags::Check).
#define gpuCheckLaunchError()                                                                              \
    {                                                                                                      \
        gpuError e = gpuGetLastError();                                                                    \
        if (e != gpuSuccess) {                                                                             \
            char buffer[256];                                                                              \
            sprintf(buffer, "GPU failure in %s:%d Message: %s", __FILE__, __LINE__, gpuGetErrorString(e)); \
            std::cerr << buffer << std::endl;                                                              \
            throw std::runtime_error(buffer);                                                              \
        }                                                                                                  \
    }

// ----------------------------------------------------------------------------

/// Compute number of blocks and threads for calculation on GPU.
/// This function calculates the number of blocks and threads for a given number of elements based on the blockSize.
void computeGridSize(uint n,           ///< total number of elements
                     uint blockSize,   ///< block size (threads per block)
                     uint& numBlocks,  ///< number of blocks [output]
                     uint& numThreads  ///< number of threads [output]
);

// ----------------------------------------------------------------------------

/// Set of error flags raised by device kernels and checked without a per-kernel host-device synchronization.
/// Each flag is one byte of device memory that kernels set to true. Reset clears all flags and Record queues
/// a copy of them into pinned host memory, both asynchronously on the default stream. Check waits for the
/// copy queued by the most recent Record (normally complete long before) and throws if any flag was set or
/// if the device reported an error. A failure is thus reported at the first Check after the Record that
/// follows the failing kernel, instead of immediately after the kernel.
class GpuErrorFlags {
  public:
    /// Create a set of flags, one per name. The names are used in the exception messages.
    GpuErrorFlags(const std::vector<std::string>& names);
    ~GpuErrorFlags();

    GpuErrorFlags(const GpuErrorFlags&) = delete;
    GpuErrorFlags& operator=(const GpuErrorFlags&) = delete;

    /// Return the device address of the flag with the given index.
    bool* Flag(int i) const { return m_flagsD + i; }

    /// Clear all flags (asynchronous).
    void Reset();

    /// Queue a copy of the flags to the host (asynchronous).
    void Record();

    /// Wait for the copy queued by the last Record and throw if a flag was set or the device reported an error.
    /// Does nothing if Record was not called since the last Check.
    void Check();

  private:
    std::vector<std::string> m_names;
    bool* m_flagsD;
    bool* m_flagsH;
    gpuEvent m_event;
    bool m_recorded;
};

/// Time recorder for GPU events.
/// This utility class encapsulates a simple timer for recording the time between a start and stop event.
class GpuTimer {
  public:
    GpuTimer(gpuStream stream = 0);
    ~GpuTimer();

    /// Record the start time.
    void Start();

    /// Record the stop time.
    void Stop();

    /// Return the elapsed time.
    float Elapsed();

  private:
    gpuStream m_stream;
    gpuEvent m_start;
    gpuEvent m_stop;
};

/// @} fsisph_utils

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
