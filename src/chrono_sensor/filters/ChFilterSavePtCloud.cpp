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

#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include <iostream>
#include <sstream>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/input_output/ChWriterCSV.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/utils/ChAsyncWriter.h"
#include <cuda_runtime_api.h>
#endif

#include <algorithm>

namespace chrono {
namespace sensor {

namespace {

void EnsureDirectoryTree(const std::string& path) {
    std::vector<std::string> split_string;
#ifdef _WIN32
    const char separator = '\\';
#else
    const char separator = '/';
#endif
    std::istringstream istring(path);
    std::string substring;
    while (std::getline(istring, substring, separator))
        split_string.push_back(substring);

    std::string partial_path;
    for (auto s : split_string) {
        if (!s.empty()) {
            partial_path += s + separator;
            if (!exists(std::filesystem::path(partial_path))) {
                if (!CreateOutputDirectory(std::filesystem::path(partial_path)))
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                else
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
            }
        }
    }
}

}  // namespace

#ifdef CHRONO_HAS_OPTIX
ChFilterSavePtCloud::ChFilterSavePtCloud(std::string data_path, std::string name) : ChFilter(name), m_path(data_path), m_num_writer_threads(ChAsyncWriter::DefaultNumThreads()) {}

// Destroying the writer waits for every pending frame to be written.
ChFilterSavePtCloud::~ChFilterSavePtCloud() {
    m_writer.reset();
}
#else
ChFilterSavePtCloud::ChFilterSavePtCloud(std::string data_path, std::string name) : ChFilter(name), m_path(data_path), m_num_writer_threads(0) {}
ChFilterSavePtCloud::~ChFilterSavePtCloud() {}
#endif

namespace {
void WritePtCloudCSV(const std::string& filename, const PixelXYZI* points, unsigned int count) {
    ChWriterCSV csv_writer(",");
    for (unsigned int i = 0; i < count; i++) {
        csv_writer << points[i].x << points[i].y << points[i].z << points[i].intensity << std::endl;
    }
    csv_writer.WriteToFile(filename);
}
}  // namespace

void ChFilterSavePtCloud::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    ++m_frame_number;
    unsigned int count = m_buffer_in->Beam_return_count;

#ifdef CHRONO_HAS_OPTIX
    // Copy the points into a free staging buffer (waits while all are in use), then format and write the CSV file on a
    // writer thread so that it does not stall the render thread.
    void* staging = m_writer->Acquire();
    cudaMemcpyAsync(staging, m_buffer_in->Buffer.get(), sizeof(PixelXYZI) * m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    m_writer->Submit(staging, [filename, count](const void* data) { WritePtCloudCSV(filename, static_cast<const PixelXYZI*>(data), count); });
#else
    m_host_buffer = m_buffer_in;
    WritePtCloudCSV(filename, m_host_buffer->Buffer.get(), count);
#endif
}

void ChFilterSavePtCloud::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

#ifdef CHRONO_HAS_OPTIX
    // Pinned staging buffers, two per writer thread, allocated on first use.
    unsigned int num_points = m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1);
    unsigned int num_buffers = std::max(1u, 2 * m_num_writer_threads);
    m_writer = chrono_types::make_shared<ChAsyncWriter>(
        m_num_writer_threads, num_buffers, [num_points]() { return std::shared_ptr<void>(cudaHostMallocHelper<PixelXYZI>(num_points), cudaHostFreeHelper<PixelXYZI>); });
#else
    m_host_buffer = m_buffer_in;
#endif

    EnsureDirectoryTree(m_path);
}

}  // namespace sensor
}  // namespace chrono
