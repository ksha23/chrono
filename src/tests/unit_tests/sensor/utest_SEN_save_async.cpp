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
// Unit tests for the background writer behind the save filters:
// - ChAsyncWriter runs every job, never holds more than its pool of staging
//   buffers, and finishes all pending jobs in its destructor.
// - A job that throws neither leaks its buffer (synchronous path) nor kills a
//   worker thread (asynchronous path).
// - Flush returns while another thread keeps submitting jobs.
// - ChFilterSave, ChFilterSavePtCloud and ChFilterRadarSavePC with writer threads
//   produce exactly the same files (names, count and bytes) as the synchronous
//   path, Flush makes every frame received so far complete on disk, and every
//   file exists once the sensor has been released.
//
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterRadarProcess.h"
#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/utils/ChAsyncWriter.h"

using namespace chrono;
using namespace chrono::sensor;

namespace fs = std::filesystem;

namespace {

std::shared_ptr<void> AllocInt() {
    return std::shared_ptr<void>(new int(0), [](void* p) { delete static_cast<int*>(p); });
}

// All regular files in a directory, by name.
std::vector<std::string> ListFiles(const fs::path& dir) {
    std::vector<std::string> names;
    if (fs::exists(dir))
        for (const auto& e : fs::directory_iterator(dir))
            if (e.is_regular_file())
                names.push_back(e.path().filename().string());
    std::sort(names.begin(), names.end());
    return names;
}

std::string ReadFile(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// Same file names in both directories, at least min_count of them, byte-identical contents, and not all empty.
void ExpectSameFiles(const fs::path& ref_dir, const fs::path& test_dir, size_t min_count) {
    auto ref = ListFiles(ref_dir);
    auto test = ListFiles(test_dir);
    EXPECT_GE(ref.size(), min_count) << ref_dir;
    ASSERT_EQ(ref, test) << ref_dir << " vs " << test_dir;
    size_t total_bytes = 0;
    for (const auto& name : ref) {
        std::string a = ReadFile(ref_dir / name);
        total_bytes += a.size();
        EXPECT_TRUE(a == ReadFile(test_dir / name)) << "contents differ: " << name;
    }
    EXPECT_GT(total_bytes, 0u) << ref_dir;
}

// After Flush on the asynchronous filter: the first n-1 frames of the synchronous reference (n files seen before the
// Flush) exist in the asynchronous directory with identical bytes.
void ExpectFramesFlushed(const fs::path& ref_dir, const fs::path& test_dir, size_t n, const std::string& ext, size_t min_count) {
    ASSERT_GE(n, min_count + 1) << ref_dir;
    for (size_t i = 0; i + 1 < n; i++) {
        std::string name = "frame_" + std::to_string(i) + ext;
        ASSERT_TRUE(fs::exists(test_dir / name)) << "not written after Flush: " << (test_dir / name);
        EXPECT_TRUE(ReadFile(ref_dir / name) == ReadFile(test_dir / name)) << "incomplete after Flush: " << name;
    }
}

}  // namespace

// Every job runs, jobs see the buffer they were given, and no more than the pool size is ever allocated or in use.
TEST(ChAsyncWriter, bounded_pool) {
    const unsigned int num_buffers = 3;
    const int num_jobs = 200;
    std::vector<int> seen(num_jobs, -1);
    {
        ChAsyncWriter writer(2, num_buffers, AllocInt);
        EXPECT_EQ(writer.GetNumAllocated(), num_buffers);  // the whole pool exists before the first frame
        for (int i = 0; i < num_jobs; i++) {
            int* staging = static_cast<int*>(writer.Acquire());
            *staging = i;
            writer.Submit(staging, [&seen, i](const void* data) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                seen[i] = *static_cast<const int*>(data);
            });
        }
        writer.Flush();
        EXPECT_LE(writer.GetNumAllocated(), num_buffers);
        EXPECT_LE(writer.GetPeakInUse(), num_buffers);
        EXPECT_GE(writer.GetPeakInUse(), 2u);  // the producer did run ahead of the writers
    }
    for (int i = 0; i < num_jobs; i++)
        EXPECT_EQ(seen[i], i);
}

// With zero threads, Submit runs the job before returning.
TEST(ChAsyncWriter, synchronous) {
    ChAsyncWriter writer(0, 1, AllocInt);
    EXPECT_EQ(writer.GetNumThreads(), 0u);
    for (int i = 0; i < 10; i++) {
        bool ran = false;
        void* staging = writer.Acquire();
        writer.Submit(staging, [&ran](const void*) { ran = true; });
        EXPECT_TRUE(ran);
    }
    EXPECT_EQ(writer.GetNumAllocated(), 1u);
}

// The destructor waits for jobs that are still queued or running.
TEST(ChAsyncWriter, destructor_flushes) {
    std::atomic<int> done{0};
    const int num_jobs = 16;
    {
        ChAsyncWriter writer(2, 4, AllocInt);
        for (int i = 0; i < num_jobs; i++) {
            void* staging = writer.Acquire();
            writer.Submit(staging, [&done](const void*) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                done++;
            });
        }
        EXPECT_LT(done.load(), num_jobs);  // work is still pending when the writer goes out of scope
    }
    EXPECT_EQ(done.load(), num_jobs);
}

// A job that throws on the synchronous path propagates the exception but still returns its buffer, so the writer can be
// used again and its destructor (which flushes) does not deadlock.
TEST(ChAsyncWriter, synchronous_exception) {
    ChAsyncWriter writer(0, 1, AllocInt);
    void* staging = writer.Acquire();
    EXPECT_THROW(writer.Submit(staging, [](const void*) { throw std::runtime_error("write failed"); }), std::runtime_error);
    ASSERT_EQ(writer.GetNumInUse(), 0u);
    staging = writer.Acquire();
    EXPECT_THROW(writer.Submit(staging, [](const void*) { throw 42; }), int);
    ASSERT_EQ(writer.GetNumInUse(), 0u);
    bool ran = false;
    staging = writer.Acquire();
    writer.Submit(staging, [&ran](const void*) { ran = true; });
    EXPECT_TRUE(ran);
    writer.Flush();
}

// A job that throws (std::exception or anything else) on a worker thread does not stop that worker or lose its buffer.
TEST(ChAsyncWriter, worker_survives_exception) {
    std::atomic<int> done{0};
    {
        ChAsyncWriter writer(1, 2, AllocInt);
        for (int i = 0; i < 10; i++) {
            void* staging = writer.Acquire();
            if (i % 3 == 0)
                writer.Submit(staging, [](const void*) { throw 7; });
            else if (i % 3 == 1)
                writer.Submit(staging, [](const void*) { throw std::runtime_error("write failed"); });
            else
                writer.Submit(staging, [&done](const void*) { done++; });
        }
        writer.Flush();
        EXPECT_EQ(writer.GetNumInUse(), 0u);
    }
    EXPECT_EQ(done.load(), 3);
}

// Flush waits for the jobs acquired before the call, and returns even though another thread keeps the pool busy.
TEST(ChAsyncWriter, flush_while_producing) {
    std::atomic<bool> stop{false};
    std::atomic<int> acquired{0};
    std::atomic<int> done{0};
    std::vector<int> finished(100000, 0);
    ChAsyncWriter writer(2, 4, AllocInt);  // declared last: its destructor runs the remaining jobs, which use the above
    std::thread producer([&]() {
        for (int i = 0; !stop && i < (int)finished.size(); i++) {
            void* staging = writer.Acquire();
            acquired++;
            writer.Submit(staging, [&finished, &done, i](const void*) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                finished[i] = 1;
                done++;
            });
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int before = acquired.load();  // every job with index < before was acquired before Flush
    writer.Flush();
    int after_flush = done.load();
    stop = true;
    producer.join();
    EXPECT_GT(before, 0);
    for (int i = 0; i < before; i++)
        EXPECT_EQ(finished[i], 1) << "job " << i << " still pending after Flush";
    EXPECT_GE(after_flush, before);
}

// Camera, lidar and radar frames written by background writers are identical to the synchronous reference, and all of
// them exist as soon as the sensors are released.
TEST(ChFilterSave, async_matches_sync) {
    const fs::path root = fs::path("utest_SEN_save_async_out");
    fs::remove_all(root);

    ChSystemNSC sys;
    auto box = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2, 1000, true, false);
    box->SetPos({4, 0.5, 0.2});
    box->SetFixed(true);
    sys.Add(box);
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(40, 40, 0.1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetFixed(true);
    sys.Add(floor);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({-10, 10, 10}, {1, 1, 1}, 500);

    ChFrame<double> pose({-2, 0, 0.5}, QUNIT);

    auto cam = chrono_types::make_shared<ChCameraSensor>(floor, 50.0f, pose, 320, 240, (float)CH_PI / 3);
    auto cam_sync = chrono_types::make_shared<ChFilterSave>((root / "cam_sync/").string());
    auto cam_async = chrono_types::make_shared<ChFilterSave>((root / "cam_async/").string());
    cam_sync->SetNumWriterThreads(0);
    cam_async->SetNumWriterThreads(3);
    cam->PushFilter(cam_sync);
    cam->PushFilter(cam_async);
    manager->AddSensor(cam);

    auto lidar = chrono_types::make_shared<ChLidarSensor>(floor, 20.0f, pose, 360, 16, (float)(2 * CH_PI), 0.2f, -0.3f, 50.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    auto pc_sync = chrono_types::make_shared<ChFilterSavePtCloud>((root / "lidar_sync/").string());
    auto pc_async = chrono_types::make_shared<ChFilterSavePtCloud>((root / "lidar_async/").string());
    pc_sync->SetNumWriterThreads(0);
    pc_async->SetNumWriterThreads(3);
    lidar->PushFilter(pc_sync);
    lidar->PushFilter(pc_async);
    manager->AddSensor(lidar);

    auto radar = chrono_types::make_shared<ChRadarSensor>(floor, 20.0f, pose, 100, 20, (float)(CH_PI / 2), 0.3f, 50.0f);
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarProcess>());
    auto radar_sync = chrono_types::make_shared<ChFilterRadarSavePC>((root / "radar_sync/").string());
    auto radar_async = chrono_types::make_shared<ChFilterRadarSavePC>((root / "radar_async/").string());
    radar_sync->SetNumWriterThreads(0);
    radar_async->SetNumWriterThreads(3);
    radar->PushFilter(radar_sync);
    radar->PushFilter(radar_async);
    manager->AddSensor(radar);

    const double step = 1e-3;
    while (sys.GetChTime() < 0.25) {
        manager->Update();
        sys.DoStepDynamics(step);
    }

    // Mid-run Flush. The synchronous filter runs just before the asynchronous one on the same frame, so if the
    // synchronous directory holds frames 0..n-1, frames 0..n-2 have already been handed to the asynchronous filter
    // and must be complete on disk once its Flush returns, while the sensors keep rendering.
    auto cam_n = ListFiles(root / "cam_sync").size();
    auto pc_n = ListFiles(root / "lidar_sync").size();
    auto radar_n = ListFiles(root / "radar_sync").size();
    cam_async->Flush();
    pc_async->Flush();
    radar_async->Flush();
    ExpectFramesFlushed(root / "cam_sync", root / "cam_async", cam_n, ".png", 5);
    ExpectFramesFlushed(root / "lidar_sync", root / "lidar_async", pc_n, ".csv", 1);
    ExpectFramesFlushed(root / "radar_sync", root / "radar_async", radar_n, ".csv", 1);

    while (sys.GetChTime() < 0.5) {
        manager->Update();
        sys.DoStepDynamics(step);
    }

    // Release everything that owns the filters: the destructors must flush the pending writes.
    manager.reset();
    cam.reset();
    lidar.reset();
    radar.reset();
    cam_sync.reset();
    cam_async.reset();
    pc_sync.reset();
    pc_async.reset();
    radar_sync.reset();
    radar_async.reset();

    ExpectSameFiles(root / "cam_sync", root / "cam_async", 10);
    ExpectSameFiles(root / "lidar_sync", root / "lidar_async", 5);
    ExpectSameFiles(root / "radar_sync", root / "radar_async", 5);

    fs::remove_all(root);
}
