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

#include "chrono_sensor/filters/ChFilterSave.h"

#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include <filesystem>
    #include <fstream>
    #include <iostream>
    #include <sstream>
    #include <vector>

    #include "chrono_thirdparty/stb/stb_image_write.h"

namespace chrono {
namespace sensor {

namespace {
bool write_rgba16_binary(const std::string& file_path, uint16_t width, uint16_t height, const void* data) {
    std::ofstream file(file_path, std::ios::binary);
    if (!file)
        return false;
    const uint16_t channels = 4;
    const uint16_t bit_depth = 16;
    file.write(reinterpret_cast<const char*>(&width), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&height), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&channels), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&bit_depth), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(data), static_cast<size_t>(width) * height * channels * sizeof(uint16_t));
    return true;
}

bool write_float_binary(const std::string& file_path, uint16_t width, uint16_t height, const void* data) {
    std::ofstream file(file_path, std::ios::binary);
    if (!file)
        return false;
    const uint16_t channels = 1;
    const uint16_t bit_depth = 32;
    file.write(reinterpret_cast<const char*>(&width), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&height), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&channels), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(&bit_depth), sizeof(uint16_t));
    file.write(reinterpret_cast<const char*>(data), static_cast<size_t>(width) * height * sizeof(float));
    return true;
}
}  // namespace

CH_SENSOR_API ChFilterSave::ChFilterSave(std::string data_path, std::string name) : ChFilter(name), m_path(data_path), m_num_writer_threads(0) {}

CH_SENSOR_API ChFilterSave::~ChFilterSave() {}

CH_SENSOR_API void ChFilterSave::Initialize(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_r8_in = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    m_rgba8_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_rgba16_in = std::dynamic_pointer_cast<SensorDeviceRGBA16Buffer>(bufferInOut);
    m_semantic_in = std::dynamic_pointer_cast<SensorHostSemanticBuffer>(bufferInOut);
    m_depth_in = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut);

    if (!m_r8_in && !m_rgba8_in && !m_rgba16_in && !m_semantic_in && !m_depth_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }

    if (!m_path.empty()) {
        std::filesystem::create_directories(std::filesystem::path(m_path));
        if (m_path.back() != '/' && m_path.back() != '\\')
            m_path += '/';
    }

    stbi_flip_vertically_on_write(1);
}

CH_SENSOR_API void ChFilterSave::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number++) + ".png";

    if (m_r8_in && m_r8_in->Buffer) {
        if (!stbi_write_png(filename.c_str(), m_r8_in->Width, m_r8_in->Height, 1, m_r8_in->Buffer.get(), m_r8_in->Width))
            std::cerr << "Failed to write R8 image: " << filename << "\n";
    } else if (m_rgba8_in && m_rgba8_in->Buffer) {
        if (!stbi_write_png(filename.c_str(), m_rgba8_in->Width, m_rgba8_in->Height, 4, m_rgba8_in->Buffer.get(),
                            static_cast<int>(sizeof(PixelRGBA8) * m_rgba8_in->Width)))
            std::cerr << "Failed to write RGBA8 image: " << filename << "\n";
    } else if (m_rgba16_in && m_rgba16_in->Buffer) {
        filename.replace(filename.length() - 3, 3, "bin");
        if (!write_rgba16_binary(filename, static_cast<uint16_t>(m_rgba16_in->Width), static_cast<uint16_t>(m_rgba16_in->Height), m_rgba16_in->Buffer.get()))
            std::cerr << "Failed to write RGBA16 image: " << filename << "\n";
    } else if (m_semantic_in && m_semantic_in->Buffer) {
        if (!stbi_write_png(filename.c_str(), m_semantic_in->Width, m_semantic_in->Height, 4, m_semantic_in->Buffer.get(),
                            static_cast<int>(sizeof(PixelSemantic) * m_semantic_in->Width)))
            std::cerr << "Failed to write semantic image: " << filename << "\n";
    } else if (m_depth_in && m_depth_in->Buffer) {
        filename.replace(filename.length() - 3, 3, "bin");
        if (!write_float_binary(filename, static_cast<uint16_t>(m_depth_in->Width), static_cast<uint16_t>(m_depth_in->Height), m_depth_in->Buffer.get()))
            std::cerr << "Failed to write depth map: " << filename << "\n";
    }
}

CH_SENSOR_API void ChFilterSave::ChangeDataPath(std::string data_path) {
    m_path = data_path;
}

}  // namespace sensor
}  // namespace chrono

#else

#include <algorithm>
#include <vector>
#include <sstream>
#include <fstream>

#include "chrono/core/ChDataPath.h"

#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/utils/ChAsyncWriter.h"

#include "chrono_thirdparty/stb/stb_image_write.h"

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

/// @brief Helper function to write an image data in RGBA 16-bit format to a binary file
/// @param file_path The path string to the output file
/// @param width The width of the image in pixels
/// @param height The height of the image in pixels
/// @param data A pointer to the image data in RGBA 16-bit format (uint16_t)
/// @return true if the file was successfully written, false otherwise
bool WriteRGBA16ToBinary(const std::string& file_path, uint16_t width, uint16_t height, const void* data) {
    
    const uint16_t channels = 4; // number of channels (RGBA)
    const uint16_t bit_depth = 16; // uint16 for each pixel channel

    // recursively create folder if not exists
    // std::std::filesystem::path path(file_path);
    // std::filesystem::create_directories(path.parent_path());

    // Open file in binary mode
    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return false;
    }
    else {
        // Write width, height, number of channels, and bit depth
        file.write(reinterpret_cast<const char*>(&width), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&height), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&channels), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&bit_depth), sizeof(uint16_t));

        // Calculate the total number of pixels
        size_t totalPixels = width * height * channels;

        // Write image data
        file.write(reinterpret_cast<const char*>(data), totalPixels * sizeof(uint16_t));

        // Close the file
        file.close();
        return true;
    }
}

/// @brief Helper function to write a float map (ex: depth map) to a binary file
/// @param file_path The path string to the output file
/// @param width The width of the image in pixels
/// @param height The height of the image in pixels
/// @param data A pointer to the image data in float format
/// @return true if the file was successfully written, false otherwise
bool WriteFloatToBinary(const std::string& file_path, uint16_t width, uint16_t height, const void* data) {
    
    const uint16_t channels = 1; // Number of channels
    const uint16_t bit_depth = 32; // bit length of FLOAT

    // recursively create folder if not exists
    // std::std::filesystem::path path(file_path);
    // std::filesystem::create_directories(path.parent_path());

    // Open file in binary mode
    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return false;
    }
    else {
        // Write width, height, number of channels, and bit depth
        file.write(reinterpret_cast<const char*>(&width), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&height), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&channels), sizeof(uint16_t));
        file.write(reinterpret_cast<const char*>(&bit_depth), sizeof(uint16_t));

        // Calculate the total number of pixels
        size_t totalPixels = width * height * channels;

        // Write image data
        file.write(reinterpret_cast<const char*>(data), totalPixels * sizeof(float));

        // Close the file
        file.close();
        return true;
    }
}

CH_SENSOR_API ChFilterSave::ChFilterSave(std::string data_path, std::string name) : ChFilter(name), m_num_writer_threads(ChAsyncWriter::DefaultNumThreads()) {
    m_path = data_path;
}

// Destroying the writer waits for every pending frame to be written.
CH_SENSOR_API ChFilterSave::~ChFilterSave() {
    m_writer.reset();
}

CH_SENSOR_API void ChFilterSave::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".png";
    m_frame_number++;

    // Copy the frame into a free staging buffer (waits while all are in use), then encode and write it on a writer
    // thread so that PNG compression does not stall the render thread.
    void* staging = m_writer->Acquire();

    if (m_r8_in) {
        unsigned int w = m_r8_in->Width;
        unsigned int h = m_r8_in->Height;
        cudaMemcpyAsync(staging, m_r8_in->Buffer.get(), w * h * sizeof(char), cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        m_writer->Submit(staging, [filename, w, h](const void* data) {
            // write a grayscale png
            if (!stbi_write_png(filename.c_str(), w, h, 1, data, w)) {
                std::cerr << "Failed to write R8 image: " << filename << "\n";
            }
        });
    } else if (m_rgba8_in) {
        unsigned int w = m_rgba8_in->Width;
        unsigned int h = m_rgba8_in->Height;
        cudaMemcpyAsync(staging, m_rgba8_in->Buffer.get(), w * h * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        m_writer->Submit(staging, [filename, w, h](const void* data) {
            // write an rgba png
            if (!stbi_write_png(filename.c_str(), w, h, sizeof(PixelRGBA8), data, sizeof(PixelRGBA8) * w)) {
                std::cerr << "Failed to write RGBA8 image: " << filename << "\n";
            }
        });
    } else if (m_rgba16_in) {
        unsigned int w = m_rgba16_in->Width;
        unsigned int h = m_rgba16_in->Height;
        cudaMemcpyAsync(staging, m_rgba16_in->Buffer.get(), w * h * sizeof(PixelRGBA16), cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        filename.replace(filename.length() - 3, 3, "bin");
        m_writer->Submit(staging, [filename, w, h](const void* data) {
            if (!WriteRGBA16ToBinary(filename, w, h, data)) {
                std::cerr << "Failed to write RGBA16 image: " << filename << "\n";
            }
        });
    } else if (m_semantic_in) {
        unsigned int w = m_semantic_in->Width;
        unsigned int h = m_semantic_in->Height;
        cudaMemcpyAsync(staging, m_semantic_in->Buffer.get(), w * h * sizeof(PixelSemantic), cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        m_writer->Submit(staging, [filename, w, h](const void* data) {
            // write an rgba png
            if (!stbi_write_png(filename.c_str(), w, h, 4, data, 4 * w)) {
                std::cerr << "Failed to write semantic image: " << filename << "\n";
            }
        });
    } else if (m_depth_in) {
        unsigned int w = m_depth_in->Width;
        unsigned int h = m_depth_in->Height;
        cudaMemcpyAsync(staging, m_depth_in->Buffer.get(), w * h * sizeof(PixelDepth), cudaMemcpyDeviceToHost, m_cuda_stream);
        cudaStreamSynchronize(m_cuda_stream);
        // write the depth map
        filename.replace(filename.length() - 3, 3, "bin");
        m_writer->Submit(staging, [filename, w, h](const void* data) {
            if (!WriteFloatToBinary(filename, w, h, data)) {
                std::cerr << "Failed to write depth map to " << filename << "\n";
            }
        });
    } else {
        m_writer->Submit(staging, [](const void*) {});
    }
}

CH_SENSOR_API void ChFilterSave::Initialize(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    // size in bytes of one staging buffer (a full copy of the input frame)
    size_t staging_bytes = 0;
    if (auto pR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut)) {
        m_r8_in = pR8;
        staging_bytes = m_r8_in->Width * m_r8_in->Height * sizeof(char);
    } else if (auto pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        m_rgba8_in = pRGBA8;
        staging_bytes = m_rgba8_in->Width * m_rgba8_in->Height * sizeof(PixelRGBA8);
    } else if (auto pRGBA16 = std::dynamic_pointer_cast<SensorDeviceRGBA16Buffer>(bufferInOut)) {
        m_rgba16_in = pRGBA16;
        staging_bytes = m_rgba16_in->Width * m_rgba16_in->Height * sizeof(PixelRGBA16);
    } else if (auto pSemantic = std::dynamic_pointer_cast<SensorDeviceSemanticBuffer>(bufferInOut)) {
        m_semantic_in = pSemantic;
        staging_bytes = m_semantic_in->Width * m_semantic_in->Height * sizeof(PixelSemantic);
    } else if (auto pDepth = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut)) {
        m_depth_in = pDepth;
        staging_bytes = m_depth_in->Width * m_depth_in->Height * sizeof(PixelDepth);
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    // Pinned staging buffers, two per writer thread, allocated on first use. This bounds the host memory held by
    // frames waiting to be written.
    unsigned int num_buffers = std::max(1u, 2 * m_num_writer_threads);
    m_writer = chrono_types::make_shared<ChAsyncWriter>(m_num_writer_threads, num_buffers, [staging_bytes]() {
        return std::shared_ptr<void>(cudaHostMallocHelper<unsigned char>(static_cast<unsigned int>(staging_bytes)), cudaHostFreeHelper<unsigned char>);
    });

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }

    std::vector<std::string> split_string;

    std::istringstream istring(m_path);

    std::string substring;
    while (std::getline(istring, substring, '/')) {
        split_string.push_back(substring);
    }

    std::string partial_path = "";
    for (auto s : split_string) {
        if (s != "") {
            partial_path += s + "/";
            if (!exists(std::filesystem::path(partial_path))) {
                if (!CreateOutputDirectory(std::filesystem::path(partial_path))) {
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                } else {
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
                }
            }
        }
    }

    if (m_path.back() != '/' && m_path != "")
        m_path += '/';

    // openGL buffers are bottom to top...so flip when writing png.
    stbi_flip_vertically_on_write(1);
}

CH_SENSOR_API void ChFilterSave::ChangeDataPath(std::string data_path) {
    m_path = data_path;
}

}  // namespace sensor
}  // namespace chrono

#endif
