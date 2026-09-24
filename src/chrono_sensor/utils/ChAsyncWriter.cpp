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
//
// Background writer used by the save filters.
//
// =============================================================================

#include "chrono_sensor/utils/ChAsyncWriter.h"

#include <algorithm>
#include <exception>
#include <iostream>

namespace chrono {
namespace sensor {

ChAsyncWriter::ChAsyncWriter(unsigned int num_threads, unsigned int num_buffers, Allocator allocator)
    : m_allocator(std::move(allocator)), m_max_buffers(std::max(1u, num_buffers)) {
    m_threads.reserve(num_threads);
    for (unsigned int i = 0; i < num_threads; i++)
        m_threads.emplace_back(&ChAsyncWriter::WorkerLoop, this);
}

ChAsyncWriter::~ChAsyncWriter() {
    Flush();
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
    }
    m_cv_task.notify_all();
    for (auto& t : m_threads)
        t.join();
}

void* ChAsyncWriter::Acquire() {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_free.empty() && m_buffers.size() < m_max_buffers) {
        m_buffers.push_back(m_allocator());
        m_free.push_back(m_buffers.back().get());
    }
    // back-pressure: wait for a writer to release a buffer rather than dropping the frame
    m_cv_free.wait(lock, [this] { return !m_free.empty(); });
    void* staging = m_free.back();
    m_free.pop_back();
    m_in_use++;
    m_peak_in_use = std::max(m_peak_in_use, m_in_use);
    return staging;
}

void ChAsyncWriter::Submit(void* staging, Job job) {
    if (m_threads.empty()) {
        job(staging);
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_free.push_back(staging);
            m_in_use--;
        }
        m_cv_idle.notify_all();
        m_cv_free.notify_one();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_tasks.push_back({staging, std::move(job)});
    }
    m_cv_task.notify_one();
}

void ChAsyncWriter::Flush() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv_idle.wait(lock, [this] { return m_in_use == 0; });
}

unsigned int ChAsyncWriter::GetNumAllocated() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<unsigned int>(m_buffers.size());
}

unsigned int ChAsyncWriter::GetPeakInUse() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_peak_in_use;
}

unsigned int ChAsyncWriter::DefaultNumThreads() {
    unsigned int hw = std::thread::hardware_concurrency();
    return std::max(1u, std::min(4u, hw / 2));
}

void ChAsyncWriter::WorkerLoop() {
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv_task.wait(lock, [this] { return m_stop || !m_tasks.empty(); });
            if (m_tasks.empty())
                return;  // stop requested and nothing left to write
            task = std::move(m_tasks.front());
            m_tasks.pop_front();
        }

        // a failed write must not take down the worker (and with it every later frame)
        try {
            task.job(task.staging);
        } catch (const std::exception& e) {
            std::cerr << "ChAsyncWriter: write failed: " << e.what() << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_free.push_back(task.staging);
            m_in_use--;
            if (m_in_use == 0)
                m_cv_idle.notify_all();
        }
        m_cv_free.notify_one();
    }
}

}  // namespace sensor
}  // namespace chrono
