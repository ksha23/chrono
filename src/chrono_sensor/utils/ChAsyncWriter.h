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
// Background writer used by the save filters: a fixed pool of staging buffers
// and a set of worker threads that encode and write files off the render thread.
//
// =============================================================================

#ifndef CHASYNCWRITER_H
#define CHASYNCWRITER_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

/// Background file writer with a bounded pool of staging buffers.
///
/// A producer (a save filter running on a render thread) calls Acquire() to get a free staging buffer, fills it, and
/// calls Submit() with a job that reads the buffer and writes a file. The job runs on one of the worker threads and the
/// buffer returns to the pool when the job finishes. Acquire() blocks while every buffer is in use, so memory stays
/// bounded and no frame is ever dropped: a producer that outruns the writers is slowed down to their pace.
/// Staging buffers are created lazily, up to the pool size, with the user-supplied allocator.
///
/// With zero worker threads the writer is synchronous: Submit() runs the job on the calling thread before returning.
/// The destructor waits for every submitted job to finish.
class CH_SENSOR_API ChAsyncWriter {
  public:
    /// Allocator for one staging buffer; the returned pointer owns the memory and frees it on release.
    using Allocator = std::function<std::shared_ptr<void>()>;

    /// Job run on a worker thread; receives the staging buffer it was submitted with.
    using Job = std::function<void(const void* staging)>;

    /// Create the writer.
    /// @param num_threads Number of worker threads (0 = run every job synchronously in Submit).
    /// @param num_buffers Maximum number of staging buffers (at least 1); also the bound on jobs in flight.
    /// @param allocator Creates one staging buffer.
    ChAsyncWriter(unsigned int num_threads, unsigned int num_buffers, Allocator allocator);

    /// Wait for all submitted jobs, then stop and join the worker threads.
    ~ChAsyncWriter();

    ChAsyncWriter(const ChAsyncWriter&) = delete;
    ChAsyncWriter& operator=(const ChAsyncWriter&) = delete;

    /// Return a free staging buffer, blocking while all buffers are in use.
    /// The caller must pass the returned buffer to exactly one Submit() call.
    void* Acquire();

    /// Queue a job that consumes a staging buffer obtained from Acquire().
    void Submit(void* staging, Job job);

    /// Block until every job submitted so far has finished.
    void Flush();

    /// Number of worker threads (0 = synchronous).
    unsigned int GetNumThreads() const { return static_cast<unsigned int>(m_threads.size()); }

    /// Number of staging buffers allocated so far (never more than the pool size given at construction).
    unsigned int GetNumAllocated();

    /// Largest number of staging buffers that were checked out at the same time.
    unsigned int GetPeakInUse();

    /// Default number of worker threads used by the save filters: min(4, half the hardware threads), at least 1.
    static unsigned int DefaultNumThreads();

  private:
    void WorkerLoop();

    struct Task {
        void* staging;
        Job job;
    };

    Allocator m_allocator;
    unsigned int m_max_buffers;

    std::mutex m_mutex;
    std::condition_variable m_cv_task;             ///< signals workers that a task is queued (or stop)
    std::condition_variable m_cv_free;             ///< signals producers that a staging buffer was released
    std::condition_variable m_cv_idle;             ///< signals Flush that the writer went idle
    std::deque<Task> m_tasks;                      ///< tasks waiting for a worker
    std::vector<std::shared_ptr<void>> m_buffers;  ///< every staging buffer allocated so far
    std::vector<void*> m_free;                     ///< staging buffers not currently checked out
    unsigned int m_in_use = 0;                     ///< staging buffers checked out (acquired, queued or running)
    unsigned int m_peak_in_use = 0;
    bool m_stop = false;

    std::vector<std::thread> m_threads;
};

/// @} sensor_utils

}  // namespace sensor
}  // namespace chrono

#endif
