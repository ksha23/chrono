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
//
// =============================================================================

#ifndef CHFILTERSAVE_H
#define CHFILTERSAVE_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/ChConfigSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include <cuda.h>
#else
    using CUstream = void*;
#endif

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;
class ChAsyncWriter;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, saves the data as an image.
///
/// With the OptiX backend, the frame is copied into a pinned staging buffer on the render thread and the image is
/// encoded and written by background writer threads, so PNG compression does not stall the sensor pipeline. At most
/// twice the number of writer threads frames are held in staging buffers; when all of them are in use the render thread
/// waits for a writer instead of dropping the frame. File names are the same as for a synchronous save. All pending
/// files are written by the time the filter is destroyed (i.e. when the sensor is released).
class CH_SENSOR_API ChFilterSave : public ChFilter {
  public:
    /// Class constructor
    /// @param data_path The path to save the data
    /// @param name the name of the filter
    ChFilterSave(std::string data_path = "", std::string name = "ChFilterSave");

    /// Class destructor. Waits until every pending frame has been written.
    virtual ~ChFilterSave();

    /// Apply function. Saves image data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// @brief Change the path of where the Save filter saves data
    /// @param data_path The new path string to save sensor data output files
    void ChangeDataPath(std::string data_path);

    /// Set the number of background writer threads (OptiX backend only). Must be called before the sensor is added
    /// to the sensor manager. 0 writes each frame synchronously on the render thread. The default is
    /// min(4, half the hardware threads).
    void SetNumWriterThreads(unsigned int num_threads) { m_num_writer_threads = num_threads; }

  private:
    std::string m_path;               ///< path to where data should be saved
    unsigned int m_frame_number = 0;  ///< frame counter to prevent overwriting data

    std::shared_ptr<SensorDeviceRGBA8Buffer> m_rgba8_in;  ///< input buffer for rgba8 image

    std::shared_ptr<SensorDeviceRGBA16Buffer> m_rgba16_in;  ///< input buffer for rgba16 image

    std::shared_ptr<SensorDeviceR8Buffer> m_r8_in;  ///< input buffer for r8 image

    std::shared_ptr<SensorHostSemanticBuffer> m_semantic_in;  ///< input buffer for semantic image

    std::shared_ptr<SensorDeviceDepthBuffer> m_depth_in;  ///< input buffer for depth map (float)

    CUstream m_cuda_stream;  ///< reference to the cuda stream

    unsigned int m_num_writer_threads;        ///< number of background writer threads (0 = synchronous)
    std::shared_ptr<ChAsyncWriter> m_writer;  ///< staging buffers + writer threads (OptiX backend)
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
