%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/ChSensorRenderTypes.h"
#include "chrono_sensor/vulkan/ChVulkanRTScene.h"

using namespace chrono;
using namespace chrono::sensor;

%}

/* The render types shared by all backends (lens model, integrator, beam shape, background).
   Without OptiX this header defines them itself, in chrono::sensor. */
%include "chrono_sensor/ChSensorRenderTypes.h"

/* Renderer-side views of the scene, not meant for simulation code. */
%ignore chrono::sensor::ChVulkanRTSceneStats;
%ignore chrono::sensor::ChVulkanRTTexCoord;
%ignore chrono::sensor::ChVulkanRTMaterial;
%ignore chrono::sensor::ChVulkanRTTriangle;
%ignore chrono::sensor::ChVulkanRTPrimitive;
%ignore chrono::sensor::ChVulkanRTLight;
%ignore chrono::sensor::ChVulkanRTScene::GetStats;
%ignore chrono::sensor::ChVulkanRTScene::GetPrimitives;
%ignore chrono::sensor::ChVulkanRTScene::GetLights;
%ignore chrono::sensor::ChVulkanRTScene::SetLights;

/* Parse the header file to generate wrappers */
%include "chrono_sensor/vulkan/ChVulkanRTScene.h"
