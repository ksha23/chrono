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
// Unit tests for the NVRTC shader cache in ChOptixUtils.
//
// - The cache key changes when the shader source, any file it includes (directly or transitively),
//   the compile options or the toolchain description change, and does not change with the current
//   working directory.
// - A cached module is byte-identical to a fresh NVRTC compile, is served from memory and from disk
//   (including from a different working directory), and a corrupt disk entry is recompiled.
// - Every NVRTC program that is created is destroyed.
//
// =============================================================================

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono_sensor/optix/ChOptixUtils.h"

using namespace chrono::sensor;
namespace fs = std::filesystem;

namespace {

void WriteText(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream f(path, std::ios::binary);
    f << text;
}

void SetEnv(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value ? value : "");
#else
    if (value)
        setenv(name, value, 1);
    else
        unsetenv(name);
#endif
}

// Temporary directory removed at the end of the test; also restores the working directory.
struct TempDir {
    fs::path path;
    fs::path old_cwd;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("chrono_utest_shader_cache_" + tag + "_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        fs::create_directories(path);
        old_cwd = fs::current_path();
    }
    ~TempDir() {
        std::error_code ec;
        fs::current_path(old_cwd, ec);
        fs::remove_all(path, ec);
    }
};

}  // namespace

TEST(ChShaderCache, KeyTracksSourcesOptionsAndToolchain) {
    TempDir tmp("key");
    const fs::path src = tmp.path / "shaders" / "main.cu";
    const fs::path inc_dir = tmp.path / "include";
    WriteText(src, "#include \"lib/a.cuh\"\n__global__ void k() {}\n");
    WriteText(inc_dir / "lib" / "a.cuh", "  #  include \"b.cuh\"\n#include <cuda_runtime.h>\nint a;\n");
    WriteText(inc_dir / "lib" / "b.cuh", "int b;\n");
    WriteText(inc_dir / "lib" / "unrelated.cuh", "int u;\n");

    const std::vector<std::string> dirs = {inc_dir.string()};
    const std::vector<std::string> opts = {"-I" + inc_dir.string(), "-use_fast_math"};
    const std::string key = ComputeShaderCacheKey(src.string(), dirs, opts, "tc");
    EXPECT_EQ(key.size(), 32u);
    EXPECT_EQ(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));

    // transitive include, found next to the including file
    WriteText(inc_dir / "lib" / "b.cuh", "int b2;\n");
    EXPECT_NE(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));
    WriteText(inc_dir / "lib" / "b.cuh", "int b;\n");
    EXPECT_EQ(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));

    // direct include, found through the include directory
    WriteText(inc_dir / "lib" / "a.cuh", "  #  include \"b.cuh\"\n#include <cuda_runtime.h>\nint a2;\n");
    EXPECT_NE(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));
    WriteText(inc_dir / "lib" / "a.cuh", "  #  include \"b.cuh\"\n#include <cuda_runtime.h>\nint a;\n");

    // main source, options and toolchain
    WriteText(src, "#include \"lib/a.cuh\"\n__global__ void k2() {}\n");
    EXPECT_NE(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));
    WriteText(src, "#include \"lib/a.cuh\"\n__global__ void k() {}\n");
    EXPECT_NE(key, ComputeShaderCacheKey(src.string(), dirs, {opts[0], opts[1], "--optix-ir"}, "tc"));
    EXPECT_NE(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc2"));

    // a file that is not included does not matter
    WriteText(inc_dir / "lib" / "unrelated.cuh", "int u2;\n");
    EXPECT_EQ(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));

    // nor does the working directory
    fs::create_directories(tmp.path / "elsewhere" / "deeper");
    fs::current_path(tmp.path / "elsewhere" / "deeper");
    EXPECT_EQ(key, ComputeShaderCacheKey(src.string(), dirs, opts, "tc"));
}

TEST(ChShaderCache, CachedModuleMatchesFreshCompile) {
    TempDir tmp("compile");
    const fs::path cache_dir = tmp.path / "cache";
    SetEnv("CHRONO_SENSOR_SHADER_CACHE_DIR", cache_dir.string().c_str());

    std::vector<bool> modes = {false};
#if CUDART_VERSION >= 12000
    modes.push_back(true);
#endif

    for (bool optixir : modes) {
        SCOPED_TRACE(optixir ? "OptiX-IR" : "PTX");
        bool hit = true;

        // reference: cache disabled, always compiled
        SetEnv("CHRONO_SENSOR_SHADER_CACHE", "0");
        EXPECT_EQ(GetShaderCacheDir(), "");
        const std::string fresh = CompileShader("box", optixir, &hit);
        EXPECT_FALSE(hit);
        ASSERT_FALSE(fresh.empty());

        // first compile with the cache on: miss, and an entry is written to disk
        SetEnv("CHRONO_SENSOR_SHADER_CACHE", nullptr);
        EXPECT_EQ(GetShaderCacheDir(), cache_dir.string());
        ClearShaderCache(true);
        EXPECT_EQ(CompileShader("box", optixir, &hit), fresh);
        EXPECT_FALSE(hit);
        std::vector<fs::path> entries;
        for (const auto& e : fs::directory_iterator(cache_dir))
            entries.push_back(e.path());
        ASSERT_EQ(entries.size(), 1u);

        // in-process hit
        auto before = GetShaderCacheStats();
        EXPECT_EQ(CompileShader("box", optixir, &hit), fresh);
        EXPECT_TRUE(hit);
        EXPECT_EQ(GetShaderCacheStats().memory_hits, before.memory_hits + 1);

        // on-disk hit, from a different working directory
        ClearShaderCache(false);
        fs::create_directories(tmp.path / "other_cwd");
        fs::current_path(tmp.path / "other_cwd");
        before = GetShaderCacheStats();
        EXPECT_EQ(CompileShader("box", optixir, &hit), fresh);
        EXPECT_TRUE(hit);
        EXPECT_EQ(GetShaderCacheStats().disk_hits, before.disk_hits + 1);
        fs::current_path(tmp.old_cwd);

        // a truncated entry is ignored and rewritten
        ClearShaderCache(false);
        fs::resize_file(entries[0], fs::file_size(entries[0]) / 2);
        before = GetShaderCacheStats();
        EXPECT_EQ(CompileShader("box", optixir, &hit), fresh);
        EXPECT_FALSE(hit);
        EXPECT_EQ(GetShaderCacheStats().compiles, before.compiles + 1);
        ClearShaderCache(false);
        EXPECT_EQ(CompileShader("box", optixir, &hit), fresh);
        EXPECT_TRUE(hit);

        ClearShaderCache(true);
    }

    // no NVRTC program outlives its compile
    const auto stats = GetShaderCacheStats();
    EXPECT_GT(stats.programs_created, 0u);
    EXPECT_EQ(stats.programs_created, stats.programs_destroyed);

    SetEnv("CHRONO_SENSOR_SHADER_CACHE_DIR", nullptr);
}
