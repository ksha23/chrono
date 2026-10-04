%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/ChSensorRenderTypes.h"
#include "chrono_sensor/metal/ChMetalRTScene.h"

using namespace chrono;
using namespace chrono::sensor;

%}

/* The render types shared by all backends (lens model, integrator, beam shape, background).
   Without OptiX this header defines them itself, in chrono::sensor. */
%include "chrono_sensor/ChSensorRenderTypes.h"

/* Renderer-side views of the scene, not meant for simulation code. */
%ignore chrono::sensor::MetalSceneLight;
%ignore chrono::sensor::ChMetalRTScene::GetLights;
%ignore chrono::sensor::ChMetalRTScene::GetRenderScene;

/* Parse the header file to generate wrappers */
%include "chrono_sensor/metal/ChMetalRTScene.h"
