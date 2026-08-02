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
// Class for managing the all sensor updates
//
// =============================================================================

#ifndef CHSENSORMANAGER_H
#define CHSENSORMANAGER_H

#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChDynamicsManager.h"
#include "chrono_sensor/sensors/ChSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixEngine.h"
    #include "chrono_sensor/optix/scene/ChScene.h"
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    #include "chrono_sensor/vulkan/ChVulkanRTEngine.h"
    #include "chrono_sensor/vulkan/ChVulkanRTScene.h"
#endif
#ifdef CHRONO_HAS_METAL_RT
    #include "chrono_sensor/metal/ChMetalRTEngine.h"
    #include "chrono_sensor/metal/ChMetalRTScene.h"
#endif

#ifdef CHRONO_FSI_SPH
    #include "chrono_sensor/ChFsiSphRender.h"
#endif

#include <fstream>
#include <sstream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor
/// @{

/// class for managing sensors. This is the Sensor system class.

class CH_SENSOR_API ChSensorManager {
  public:
    /// Class constructor.
    /// The chrono system with which the sensor manager is associated is used for time management.
    ChSensorManager(ChSystem* chrono_system);

    ~ChSensorManager();

    /// Update the sensors as needed according to the current time of the chrono simulation.
    void Update();

    /// Add a sensor to the manager.
    /// @param sensor The sensor that should be added to the system
    void AddSensor(std::shared_ptr<ChSensor> sensor);

    /// Get the list of sensors for which this manager is responsible.
    /// @return The list of sensors for which the manager is responsible and updates
    std::vector<std::shared_ptr<ChSensor>> GetSensorList() { return m_sensor_list; }

    /// Set the list of devices (GPUs) that should be used for rendering.
    /// @param device_ids List of IDs corresponding to the devices (GPUs) that should be used.
    void SetDeviceList(std::vector<unsigned int> device_ids);

    /// Get the list of devices that are intended for use
    /// @return List of device IDs that the manager will try to use when rendering.
    std::vector<unsigned int> GetDeviceList();

#ifdef CHRONO_HAS_OPTIX
    /// Get the number of engines the manager is currently using.
    /// @return An integer number of OptiX engines
    int GetNumEngines() { return (int)m_engines.size(); }

    /// Get a pointer to the engine based on the id of the engine.
    /// @param context_id The ID of the engine to be returned
    /// @return A shared pointer to an OptiX engine the manager is using
    std::shared_ptr<ChOptixEngine> GetEngine(int context_id);
#elif defined(CHRONO_HAS_VULKAN_RT)
    /// Get the number of render engines the manager is currently using.
    /// Preserves the OptiX-era count API for Vulkan-only builds.
    int GetNumEngines() { return (int)m_vulkan_engines.size(); }
#elif defined(CHRONO_HAS_METAL_RT)
    /// Get the number of render engines the manager is currently using.
    /// Preserves the OptiX-era count API for Metal-only builds.
    int GetNumEngines() { return (int)m_metal_engines.size(); }
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    /// Get the number of Vulkan RT engines the manager is currently using.
    int GetNumVulkanEngines() { return (int)m_vulkan_engines.size(); }

    /// Get a pointer to a Vulkan RT engine based on its id.
    std::shared_ptr<ChVulkanRTEngine> GetVulkanEngine(int context_id);
#endif
#ifdef CHRONO_HAS_METAL_RT
    /// Get the number of Metal RT engines the manager is currently using.
    int GetNumMetalEngines() { return (int)m_metal_engines.size(); }

    /// Get a pointer to a Metal RT engine based on its id.
    std::shared_ptr<ChMetalRTEngine> GetMetalEngine(int context_id);
#endif

    /// Calls on the sensor manager to rebuild the scene.
    /// This translates all objects from the Chrono system into their active render-backend objects.
    void ReconstructScenes();

#ifdef CHRONO_FSI_SPH
    /// Attach a Chrono::FSI::SPH system for native Sensor rendering.
    /// Returns a handle that can be used to detach the source later.
    int AttachFsiSphSystem(std::shared_ptr<chrono::fsi::sph::ChFsiFluidSystemSPH> sys, const ChFsiSphRenderOptions& options = ChFsiSphRenderOptions());

    /// Detach a previously attached Chrono::FSI::SPH render source.
    void DetachFsiSphSystem(int handle);

    /// Remove all Chrono::FSI::SPH render sources from this manager.
    void ClearFsiSphSystems();
#endif

    /// Get the maximum number of allowed render engines for the manager.
    /// @return An integer specifying the maximum number of engines the manager is allowed to create.
    int GetMaxEngines() { return m_allowable_groups; }

    /// Set the maximum number of allowable render engines.
    /// The manager will spawn up to this number of render engines based on the update
    /// rate of the sensors. Sensors with similar update rates will be grouped on the same engine to reduce the number
    /// of scene updates that are required as this is a major bottleneck in the multi-threading paradigm of the render
    /// engine.
    /// @param num_groups The maximum number of render engines the manager is allowed to create.
    void SetMaxEngines(int num_groups);

    /// Set the number of recursions for ray tracing.
    /// @param rec The max number of recursions allowed in ray tracing
    void SetRayRecursions(int rec);

    /// Get the number of recursions used in ray tracing.
    /// @return The max number of recursions used in ray tracing
    int GetRayRecursions() { return m_optix_reflections; }

    /// Enable/disable verbose output mode (default: false).
    /// @param verbose whether the framework should print info
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Get the verbose setting.
    /// @return the verbose setting
    bool GetVerbose() { return m_verbose; }

    /// Enable/disable sensor debug mode (default: false).
    void SetDebug(bool debug) { m_debug = debug; }

#ifdef CHRONO_HAS_OPTIX
    /// Public pointer to the OptiX scene.
    /// This is used to specify additional components including lights, background colors, etc.
    std::shared_ptr<ChScene> scene;
#elif defined(CHRONO_HAS_VULKAN_RT)
    /// Public scene pointer preserved for OptiX-compatible demos when Vulkan RT is the render backend.
    std::shared_ptr<ChVulkanRTScene> scene;
#elif defined(CHRONO_HAS_METAL_RT)
    /// Public scene pointer preserved for OptiX-compatible demos when Metal RT is the render backend.
    std::shared_ptr<ChMetalRTScene> scene;
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    /// Public pointer to the Vulkan RT scene staging object.
    std::shared_ptr<ChVulkanRTScene> vulkan_scene;
#endif
#ifdef CHRONO_HAS_METAL_RT
    /// Public pointer to the Metal RT scene staging object.
    std::shared_ptr<ChMetalRTScene> metal_scene;
#endif

  private:
    bool m_verbose;           ///< enable printing of messages and warnings
    bool m_debug;             ///< enable debug options in sensors (if supported)
    int m_optix_reflections;  ///< maximum number of ray tracing recursions
    int m_num_keyframes;      ///< number of keyframes to use

    // class variables
    ChSystem* m_system;                                     ///< Chrono system the manager is attached to
    std::shared_ptr<ChDynamicsManager> m_dynamics_manager;  ///< container for updating dynamic sensors
#ifdef CHRONO_HAS_OPTIX
    std::vector<std::shared_ptr<ChOptixEngine>> m_engines;  ///< The optix engine(s) used for rendered sensors
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    std::vector<std::shared_ptr<ChVulkanRTEngine>> m_vulkan_engines;  ///< Vulkan RT engine(s) used for rendered sensors
#endif
#ifdef CHRONO_HAS_METAL_RT
    std::vector<std::shared_ptr<ChMetalRTEngine>> m_metal_engines;  ///< Metal RT engine(s) used for rendered sensors
#endif

    int m_allowable_groups = 1;  ///< default maximum number of allowable engines

    std::vector<unsigned int> m_device_list;                  ///< list of device IDs to use in rendering
    std::vector<std::shared_ptr<ChSensor>> m_sensor_list;     ///< list of all sensors
    std::vector<std::shared_ptr<ChSensor>> m_dynamic_sensor;  ///< list of dynamic sensors
    std::vector<std::shared_ptr<ChSensor>> m_render_sensor;   ///< list of rendered sensors
};

/// @} sensor

}  // namespace sensor
}  // namespace chrono

#endif
