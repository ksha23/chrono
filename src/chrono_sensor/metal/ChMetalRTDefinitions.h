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
// Authors: (Metal RT backend)
// =============================================================================
// Metal ray-tracing backend definitions. Mirrors ChVulkanRTDefinitions.h so the
// manager-facing sensor/pipeline concepts are identical across backends.
// =============================================================================

#ifndef CH_METAL_RT_DEFINITIONS_H
#define CH_METAL_RT_DEFINITIONS_H

#include <cstdint>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// Rendered-sensor pipeline kinds handled by the Metal RT backend.
/// Same set as the Vulkan backend's VulkanPipelineType.
enum class MetalPipelineType {
    CAMERA,
    PHYS_CAMERA,
    SEGMENTATION,
    DEPTH_CAMERA,
    NORMAL_CAMERA,
    LIDAR_SINGLE,
    LIDAR_MULTI,
    RADAR,
};

struct CH_SENSOR_API ChMetalRTDeviceConfig {
    uint32_t device_index = 0;
    bool verbose = false;
};

}  // namespace sensor
}  // namespace chrono

#endif
