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
/// All staging buffers are created up front, in the constructor, with the user-supplied allocator, so no allocation
/// happens on the producer's thread while frames are being written.
///
/// With zero worker threads the writer is synchronous: Submit() runs the job on the calling thread before returning,
/// and an exception thrown by the job propagates to the caller after the buffer is returned to the pool. On a worker
/// thread, an exception thrown by a job is reported on std::cerr and the worker moves on to the next job.
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

    /// Block until every job whose staging buffer was acquired before this call has finished. Jobs started by
    /// other threads after the call do not delay it. Safe to call from any thread.
    void Flush();

    /// Number of worker threads (0 = synchronous).
    unsigned int GetNumThreads() const { return static_cast<unsigned int>(m_threads.size()); }

    /// Number of staging buffers allocated (the pool size given at construction).
    unsigned int GetNumAllocated();

    /// Number of staging buffers currently checked out (acquired, queued or being written).
    unsigned int GetNumInUse();

    /// Largest number of staging buffers that were checked out at the same time.
    unsigned int GetPeakInUse();

    /// Default number of worker threads used by the save filters: min(4, half the hardware threads), at least 1.
    static unsigned int DefaultNumThreads();

  private:
    void WorkerLoop();

    /// Return a staging buffer to the pool once its job has finished (or thrown).
    void Release(void* staging);

    /// True if a buffer acquired before the given ticket is still checked out (caller holds the mutex).
    bool HasPendingBefore(unsigned long long ticket) const;

    struct Task {
        void* staging;
        Job job;
    };

    std::mutex m_mutex;
    std::condition_variable m_cv_task;             ///< signals workers that a task is queued (or stop)
    std::condition_variable m_cv_free;             ///< signals producers that a staging buffer was released
    std::condition_variable m_cv_idle;             ///< signals Flush that a staging buffer was released
    std::deque<Task> m_tasks;                      ///< tasks waiting for a worker
    std::vector<std::shared_ptr<void>> m_buffers;  ///< all staging buffers
    std::vector<void*> m_free;                     ///< staging buffers not currently checked out
    unsigned int m_peak_in_use = 0;
    unsigned long long m_next_ticket = 0;                             ///< sequence number of the next Acquire()
    std::vector<std::pair<void*, unsigned long long>> m_checked_out;  ///< buffers acquired, queued or being written
    bool m_stop = false;

    std::vector<std::thread> m_threads;
};

/// @} sensor_utils

}  // namespace sensor
}  // namespace chrono

#endif
