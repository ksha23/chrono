%{

/* Includes the header in the wrapper code */
#if defined(CHRONO_HAS_OPTIX)
  #include "chrono_sensor/sensors/ChOptixSensor.h"
#elif defined(CHRONO_HAS_VULKAN_RT)
  #include "chrono_sensor/sensors/ChVulkanSensor.h"
#elif defined(CHRONO_HAS_METAL_RT)
  #include "chrono_sensor/sensors/ChMetalSensor.h"
#endif
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::ChOptixSensor)
%shared_ptr(chrono::sensor::ChVulkanSensor)
%shared_ptr(chrono::sensor::ChMetalSensor)
%shared_ptr(chrono::sensor::ChCameraSensor)
%shared_ptr(chrono::sensor::ChSegmentationCamera)
%shared_ptr(chrono::sensor::ChDepthCamera)
%shared_ptr(chrono::sensor::ChLidarSensor)
%shared_ptr(chrono::sensor::ChNormalCamera)
%shared_ptr(chrono::sensor::ChRadarSensor)
%shared_ptr(chrono::sensor::ChPhysCameraSensor)

/* Parse the header file to generate wrappers. The sensors derive from the base class of the
   active backend, which carries the pipeline type next to the image size. */
#if defined(CHRONO_HAS_OPTIX)
  %include "../../../chrono_sensor/sensors/ChOptixSensor.h"
#elif defined(CHRONO_HAS_VULKAN_RT)
  %include "../../../chrono_sensor/vulkan/ChVulkanRTDefinitions.h"
  %include "../../../chrono_sensor/sensors/ChVulkanSensor.h"
#elif defined(CHRONO_HAS_METAL_RT)
  %include "../../../chrono_sensor/metal/ChMetalRTDefinitions.h"
  %include "../../../chrono_sensor/sensors/ChMetalSensor.h"
#endif
%include "../../../chrono_sensor/sensors/ChCameraSensor.h"    
%include "../../../chrono_sensor/sensors/ChSegmentationCamera.h"
%include "../../../chrono_sensor/sensors/ChDepthCamera.h"
%include "../../../chrono_sensor/sensors/ChLidarSensor.h"
%include "../../../chrono_sensor/sensors/ChNormalCamera.h"
%include "../../../chrono_sensor/sensors/ChRadarSensor.h"
%include "../../../chrono_sensor/sensors/ChPhysCameraSensor.h"

%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChCameraSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChSegmentationCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChDepthCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChLidarSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChNormalCamera)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChRadarSensor)
%DefSharedPtrDynamicCast(chrono::sensor, ChSensor, ChPhysCameraSensor)
