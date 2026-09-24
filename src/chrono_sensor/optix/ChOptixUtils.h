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
// Authors: Asher Elmquist
// =============================================================================
//
// utility functions used for optix convenience
//
// =============================================================================

#ifndef CHOPTIXUTILS_H
#define CHOPTIXUTILS_H

#include <optix.h>
#include <cuda_runtime_api.h>
#include <nvrtc.h>
#include <chrono>
#include <string>
#include <vector>

#include "chrono_sensor/ChApiSensor.h"

#include "chrono/core/ChMatrix33.h"

#include "chrono_thirdparty/stb/stb_image.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

/// Checks the output of an optix call for any error, will throw a runtime error if not success
#define OPTIX_ERROR_CHECK(result)                                                                    \
    {                                                                                                \
        if (result != OPTIX_SUCCESS) {                                                               \
            std::string error_name = std::string(optixGetErrorName(result));                         \
            std::string error_string = std::string(optixGetErrorString(result));                     \
            std::string file = std::string(__FILE__);                                                \
            std::string line = std::to_string(__LINE__);                                             \
            throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line); \
        }                                                                                            \
    }

/// Checks the output of a cuda call for any error, will throw a runtime error if not success
#define CUDA_ERROR_CHECK(result)                                                                     \
    {                                                                                                \
        if (result != cudaSuccess) {                                                                 \
            std::string error_name = std::string(cudaGetErrorName(result));                          \
            std::string error_string = std::string(cudaGetErrorString(result));                      \
            std::string file = std::string(__FILE__);                                                \
            std::string line = std::to_string(__LINE__);                                             \
            printf("Err: %s\n", error_string.c_str());                                                \
            throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line); \
        }                                                                                            \
    }

#ifdef USE_CUDA_NVRTC
    /// Checks the output of a cuda call for any error, will throw a runtime error if not success
    #define NVRTC_ERROR_CHECK(result)                                                                    \
        {                                                                                                \
            if (result != NVRTC_SUCCESS) {                                                               \
                std::string error_name = "NVRTC ERROR";                                                  \
                std::string error_string = std::string(nvrtcGetErrorString(result));                     \
                std::string file = std::string(__FILE__);                                                \
                std::string line = std::to_string(__LINE__);                                             \
                throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line); \
            }                                                                                            \
        }
#endif

/// Holds string values for ptx file and ray generation program.
struct ProgramString {
    std::string file_name;
    std::string program_name;
};

/// Stores image data.
struct ByteImageData {
    /// image width
    int w;
    /// image height
    int h;
    ///
    int c;
    /// image pixel values
    std::vector<unsigned char> data;
};

/// Launches ray generation program.
/// @param context optix device context
/// @param module optix module that will be created
/// @param file_name the file where the shader program is implemented
/// @param module_compile_options compile options for the module
/// @param pipeline_compile_options compile options for the pipeline
CH_SENSOR_API void GetShaderFromFile(OptixDeviceContext context,
                                     OptixModule& module,
                                     const std::string& file_name,
                                     OptixModuleCompileOptions& module_compile_options,
                                     OptixPipelineCompileOptions& pipeline_compile_options);

// -----------------------------------------------------------------------------
// Run-time shader compilation cache (used when shaders are compiled with NVRTC).
//
// Compiling the shader modules with NVRTC takes several seconds per process and was repeated for
// every ChOptixEngine. The compiled PTX or OptiX-IR is therefore cached in memory for the life of the
// process and on disk across processes. The key hashes the shader source, the contents of every file
// it includes with #include "..." (recursively), the NVRTC options including the include directories,
// the NVRTC and OptiX versions and the device compute capability. Angle-bracket includes (CUDA and
// OptiX headers) are covered by those version numbers and include paths rather than by their contents.
//
// Serving the cached bytes also makes the OptiX-IR independent of the current working directory,
// which NVRTC embeds in the IR; otherwise the OptiX disk cache misses whenever the cwd changes.
//
// Environment variables:
//   CHRONO_SENSOR_SHADER_CACHE      "0", "off" or "false" disables both caches; "clear" deletes the
//                                   on-disk entries once, at the first compile in the process.
//   CHRONO_SENSOR_SHADER_CACHE_DIR  on-disk cache location. Default: $XDG_CACHE_HOME or ~/.cache
//                                   (%LOCALAPPDATA% on Windows), subdirectory chrono/sensor_shaders.
// -----------------------------------------------------------------------------

/// Counters for the shader cache, accumulated over the life of the process.
struct ShaderCacheStats {
    unsigned long long memory_hits = 0;         ///< modules served from the in-process cache
    unsigned long long disk_hits = 0;           ///< modules served from the on-disk cache
    unsigned long long compiles = 0;            ///< modules compiled with NVRTC
    unsigned long long programs_created = 0;    ///< nvrtcCreateProgram calls
    unsigned long long programs_destroyed = 0;  ///< nvrtcDestroyProgram calls
};

/// Return the compiled PTX (emit_optixir = false) or OptiX-IR (emit_optixir = true) for the named shader
/// in the shader directory, compiling it with NVRTC only on a cache miss. If 'cache_hit' is given, it is
/// set to true when the result came from either cache. Throws if Chrono::Sensor was built without NVRTC.
CH_SENSOR_API std::string CompileShader(const std::string& file_name, bool emit_optixir, bool* cache_hit = nullptr);

/// Cache key for a shader source file: a hash of its contents, the contents of its transitive
/// #include "..." files (resolved relative to the including file, then against 'include_dirs'), the
/// compiler 'options' and the free-form 'toolchain' description. Independent of the working directory
/// when the paths passed in are absolute.
CH_SENSOR_API std::string ComputeShaderCacheKey(const std::string& source_file,
                                                const std::vector<std::string>& include_dirs,
                                                const std::vector<std::string>& options,
                                                const std::string& toolchain);

/// Directory of the on-disk shader cache, or an empty string if the cache is disabled.
CH_SENSOR_API std::string GetShaderCacheDir();

/// Empty the in-process shader cache and, if 'disk' is true, delete the on-disk entries.
CH_SENSOR_API void ClearShaderCache(bool disk = true);

/// Shader cache counters for this process.
CH_SENSOR_API ShaderCacheStats GetShaderCacheStats();

CH_SENSOR_API void optix_log_callback(unsigned int level, const char* tag, const char* message, void*);

/*
#ifdef USE_CUDA_NVRTC
///Launches ray generation program.
///- context optix device context
///- module optix module that will be created
///- file_name the file where the shader program is implemented
///- module_compile_options compile options for the module
///- pipeline_compile_options compile options for the pipeline
CH_SENSOR_API void GetShaderFromPtx(OptixDeviceContext context,
                                    OptixModule& module,
                                    std::string file_name,
                                    OptixModuleCompileOptions& module_compile_options,
                                    OptixPipelineCompileOptions& pipeline_compile_options);
#endif
*/

/// Loads image to struct ByteImageData, returns an empty struct with 0 values if loading failed.
/// @param filename
CH_SENSOR_API ByteImageData LoadByteImage(const std::string& filename);

/*
/// Creates an empty optix transform::node
/// @param context the optix context
optix::Transform CreateEmptyTransform(optix::Context context);

/// creates an optix::transform node
/// @param context optix context
/// @param a projection matrix
/// @param b
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector3d b);

/// creates an optix::transform node
/// @param context optix context
/// @param a  projection matrix
/// @param b
/// @param s
/// @return an optix::transform
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector3d b, ChVector3d s);

/// creates an optix::transform node based on end points
/// @param context optix context
/// @param a projection matrix
/// @param b
/// @param from
/// @return an optix::transform
optix::Transform CreateTransformFromEndPoints(optix::Context context, ChVector3d a, ChVector3d b, ChVector3d from);

/// creates an optix::transform node based on end points
/// @param context optix context
/// @param a projection matrix
/// @param b
/// @param from
/// @param s
/// @return an optix::transform
optix::Transform CreateTransformFromEndPoints(optix::Context context,
                                              ChVector3d a,
                                              ChVector3d b,
                                              ChVector3d from,
                                              ChVector3d s);

/// updates the projection matrix in the optix::transform object
/// @param t optix transform object
/// @param a projection matrix
/// @param b
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector3d b);

/// updates the projection matrix in the optix::transform object
/// @param t optix transform object
/// @param a projection matrix
/// @param b
/// @param s
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector3d b, ChVector3d s);
*/

CH_SENSOR_API void SetSensorShaderDir(const std::string& path);

CH_SENSOR_API const std::string& GetSensorShaderDir();

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif
