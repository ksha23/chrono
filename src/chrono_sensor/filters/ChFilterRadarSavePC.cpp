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

#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"

#include <iostream>
#include <sstream>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_sensor/utils/ChAsyncWriter.h"

#include <algorithm>
#include <cstring>

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <cuda_runtime_api.h>
#endif

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

ChFilterRadarSavePC::ChFilterRadarSavePC(std::string data_path, std::string name) : ChFilter(name), m_path(data_path), m_num_writer_threads(ChAsyncWriter::DefaultNumThreads()) {}

// Destroying the writer waits for every pending frame to be written.
ChFilterRadarSavePC::~ChFilterRadarSavePC() {
    m_writer.reset();
}

void ChFilterRadarSavePC::SetNumWriterThreads(unsigned int num_threads) {
    if (m_writer) {
        std::cerr << "ChFilterRadarSavePC::SetNumWriterThreads: ignored, the filter is already initialized\n";
        return;
    }
    m_num_writer_threads = num_threads;
}

void ChFilterRadarSavePC::Flush() {
    if (m_writer)
        m_writer->Flush();
}

void ChFilterRadarSavePC::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    ++m_frame_number;
    size_t max_returns = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;  // staging buffer capacity
    int count = static_cast<int>(std::min<size_t>(std::max(0, m_buffer_in->Beam_return_count), max_returns));

    // Copy the returns into a free staging buffer (waits while all are in use), then format and write the CSV file on
    // a writer thread so that it does not stall the render thread.
    void* staging = m_writer->Acquire();
    std::memcpy(staging, m_buffer_in->Buffer.get(), count * sizeof(RadarXYZReturn));
    m_writer->Submit(staging, [filename, count](const void* data) {
        const RadarXYZReturn* ret = static_cast<const RadarXYZReturn*>(data);
        ChWriterCSV csv_writer(",");
        for (int i = 0; i < count; i++) {
            csv_writer << ret[i].x << ret[i].y << ret[i].z << ret[i].vel_x << ret[i].vel_y << ret[i].vel_z << ret[i].amplitude << ret[i].objectId << std::endl;
        }
        csv_writer.WriteToFile(filename);
    });
}

void ChFilterRadarSavePC::Initialize(std::shared_ptr<ChSensor> pSensor,
                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarXYZBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pRadar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    // Staging buffers, two per writer thread, all allocated here. Each holds a full buffer of returns.
    size_t max_returns = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    unsigned int num_buffers = std::max(1u, 2 * m_num_writer_threads);
    m_writer = chrono_types::make_shared<ChAsyncWriter>(
        m_num_writer_threads, num_buffers, [max_returns]() { return std::shared_ptr<void>(new RadarXYZReturn[max_returns], std::default_delete<RadarXYZReturn[]>()); });

    EnsureDirectoryTree(m_path);
}

}  // namespace sensor
}  // namespace chrono
