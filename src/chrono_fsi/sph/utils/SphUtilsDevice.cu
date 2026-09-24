// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Radu Serban
// =============================================================================
//
// Utilities for GPU error testing and GPU timing
//
// =============================================================================

#include <algorithm>

#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

GpuTimer::GpuTimer(gpuStream stream) : m_stream(stream) {
    gpuEventCreate(&m_start);
    gpuEventCreate(&m_stop);
}

GpuTimer::~GpuTimer() {
    gpuEventDestroy(m_start);
    gpuEventDestroy(m_stop);
}

void GpuTimer::Start() {
    gpuEventRecord(m_start, m_stream);
}

void GpuTimer::Stop() {
    gpuEventRecord(m_stop, m_stream);
}

float GpuTimer::Elapsed() {
    float elapsed;
    gpuEventSynchronize(m_stop);
    gpuEventElapsedTime(&elapsed, m_start, m_stop);
    return elapsed;
}

GpuErrorFlags::GpuErrorFlags(const std::vector<std::string>& names) : m_names(names), m_flagsD(nullptr), m_flagsH(nullptr), m_recorded(false) {
    size_t n = m_names.size();
    gpuError err = gpuMalloc((void**)&m_flagsD, n * sizeof(bool));
    if (err == gpuSuccess)
        err = gpuMallocHost((void**)&m_flagsH, n * sizeof(bool));
    if (err == gpuSuccess)
        err = gpuMemset(m_flagsD, 0, n * sizeof(bool));
    if (err == gpuSuccess)
        err = gpuEventCreate(&m_event);
    if (err != gpuSuccess)
        gpuThrowError(gpuGetErrorString(err));
    std::fill(m_flagsH, m_flagsH + n, false);
}

GpuErrorFlags::~GpuErrorFlags() {
    gpuEventDestroy(m_event);
    gpuFreeHost(m_flagsH);
    gpuFree(m_flagsD);
}

void GpuErrorFlags::Reset() {
    gpuError err = gpuMemsetAsync(m_flagsD, 0, m_names.size() * sizeof(bool), 0);
    if (err != gpuSuccess)
        gpuThrowError(gpuGetErrorString(err));
}

void GpuErrorFlags::Record() {
    gpuError err = gpuMemcpyAsync(m_flagsH, m_flagsD, m_names.size() * sizeof(bool), gpuMemcpyDeviceToHost, 0);
    if (err == gpuSuccess)
        err = gpuEventRecord(m_event, 0);
    if (err != gpuSuccess)
        gpuThrowError(gpuGetErrorString(err));
    m_recorded = true;
}

void GpuErrorFlags::Check() {
    if (!m_recorded)
        return;
    m_recorded = false;

    gpuError err = gpuEventSynchronize(m_event);
    if (err != gpuSuccess)
        gpuThrowError(gpuGetErrorString(err));
    for (size_t i = 0; i < m_names.size(); i++) {
        if (m_flagsH[i]) {
            char buffer[256];
            sprintf(buffer, "Error flag intercepted from %s", m_names[i].c_str());
            std::cerr << buffer << std::endl;
            throw std::runtime_error(buffer);
        }
    }
    gpuCheckLaunchError();
}

void computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads) {
    uint n2 = (n == 0) ? 1 : n;
    numThreads = min(blockSize, n2);
    numBlocks = (n2 % numThreads != 0) ? (n2 / numThreads + 1) : (n2 / numThreads);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
