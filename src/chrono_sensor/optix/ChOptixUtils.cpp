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
// utility functions used for optix convenience
//
// =============================================================================

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#include <optix_stubs.h>
#ifdef _WIN32
    #include <process.h>  // _getpid
#else
    #include <dlfcn.h>   // dladdr
    #include <unistd.h>  // getpid
#endif
// #include <optix_function_table_definition.h>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

#ifdef USE_CUDA_NVRTC
    #include <cuda.h>  // for CUDA_VERSION
    #include <nvrtc.h>

// Whether to ASK NVRTC for an OptiX-IR module rather than PTX. This selects what is tried first;
// GetShaderFromFile below falls back to PTX at run time if the driver rejects the IR.
//
// OptiX-IR is preferred where it works: on Blackwell / RTX 50-series GPUs (sm_120) the driver's
// OptiX PTX front end has been seen to abort inside optixModuleCreate, while OptiX-IR compiles
// cleanly.
//
// NVRTC gained --optix-ir and nvrtcGetOptiXIR in CUDA 12.0; CUDA 11.8 has neither, and Chrono
// declares no minimum CUDA version, so below that floor only PTX is possible. That is exactly
// what every build did before OptiX-IR was introduced, so nothing regresses on older toolkits.
//
// This cannot be more than a first choice. NVRTC (the toolkit) produces the IR but libnvoptix
// (the display driver) consumes it, and OptiX refuses IR newer than its own front end, so a
// toolkit ahead of the driver fails at run time however new both are: CUDA 13.3 emits NVVM IR
// 115 and a 580-series driver accepts at most 98. The two are versioned independently and a
// container makes them independently installable, so the working combination has to be found by
// trying rather than asserted here.
//
// OptiX needs no companion check. OptiX-IR input landed in OptiX 7.5, and Chrono already calls
// optixModuleCreate rather than the older optixModuleCreateFromPTX, so it requires 7.7 or newer
// regardless. optixModuleCreate detects the buffer format itself and is passed an explicit size,
// so the call below is identical for either input.
    #if CUDA_VERSION >= 12000
        #define CH_OPTIX_EMIT_OPTIXIR 1
    #else
        #define CH_OPTIX_EMIT_OPTIXIR 0
    #endif
#endif

namespace chrono {
namespace sensor {

static std::string shader_dir = CHRONO_SENSOR_SHADER_DIR;

void SetSensorShaderDir(const std::string& path) {
    shader_dir = path;
}

const std::string& GetSensorShaderDir() {
    return shader_dir;
}

// -----------------------------------------------------------------------------
// Shader cache
// -----------------------------------------------------------------------------

namespace {

// Bumped whenever the on-disk format or the meaning of the key changes, so old entries are never read.
const char* const kShaderCacheFormat = "chrono-sensor-nvrtc-cache-2";
const char kShaderCacheMagic[8] = {'C', 'H', 'N', 'V', 'R', 'T', 'C', '1'};

// Two 64-bit lanes: FNV-1a and a rotate-multiply mix. Not cryptographic; entries are also validated
// by length and payload checksum when read back.
struct ShaderHash {
    uint64_t a = 0xcbf29ce484222325ull;
    uint64_t b = 0x9e3779b97f4a7c15ull;
    void Add(const void* data, size_t n) {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < n; i++) {
            a = (a ^ p[i]) * 0x100000001b3ull;
            b = b ^ p[i];
            b = ((b << 23) | (b >> 41)) * 0xff51afd7ed558ccdull;
        }
    }
    void Add(const std::string& s) {
        uint64_t n = s.size();
        Add(&n, sizeof(n));
        Add(s.data(), s.size());
    }
    std::string Hex() const {
        std::ostringstream os;
        os << std::hex << std::setfill('0') << std::setw(16) << a << std::setw(16) << b;
        return os.str();
    }
};

uint64_t PayloadChecksum(const std::string& s) {
    ShaderHash h;
    h.Add(s);
    return h.a;
}

bool ReadFileBytes(const std::filesystem::path& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good())
        return false;
    std::stringstream buffer;
    buffer << f.rdbuf();
    out = buffer.str();
    return true;
}

// Size and modification time of a file, plus the path it resolves to, or "<missing>".
std::string FileStamp(const std::filesystem::path& path) {
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    if (ec)
        return "<missing>";
    // cast: the tick count is a 128-bit integer in libc++, which ostream does not print
    const long long mtime = static_cast<long long>(std::filesystem::last_write_time(path, ec).time_since_epoch().count());
    const auto resolved = std::filesystem::weakly_canonical(path, ec);
    std::ostringstream os;
    os << (ec ? path : resolved).generic_string() << " size=" << size << " mtime=" << mtime;
    return os.str();
}

// Hash the contents of every file reached through #include "..." from 'content', recursively. Quoted
// includes are resolved the way NVRTC resolves them: next to the including file first, then in the -I
// directories. Directives inside inactive #if blocks are followed too, which can only add files to the
// key. An unresolved include is hashed by name, so a file appearing later still changes the key.
// An #include <...> found in the -I directories (OptiX, NanoVDB, CUDA headers) is hashed by its
// resolved path, size and modification time, not by its contents, and is not followed further; one
// that is not found there (an NVRTC built-in header) is hashed by name only.
void HashQuotedIncludes(const std::filesystem::path& file,
                        const std::string& content,
                        const std::vector<std::string>& include_dirs,
                        ShaderHash& hash,
                        std::set<std::string>& visited) {
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        size_t i = line.find_first_not_of(" \t");
        if (i == std::string::npos || line[i] != '#')
            continue;
        i = line.find_first_not_of(" \t", i + 1);
        if (i == std::string::npos || line.compare(i, 7, "include") != 0)
            continue;
        i = line.find_first_not_of(" \t", i + 7);
        if (i == std::string::npos || (line[i] != '"' && line[i] != '<'))
            continue;
        const bool angle = (line[i] == '<');
        const size_t close = line.find(angle ? '>' : '"', i + 1);
        if (close == std::string::npos)
            continue;
        const std::string name = line.substr(i + 1, close - i - 1);

        if (angle) {
            hash.Add("<" + name + ">");
            std::string stamp = "<system>";
            for (const auto& dir : include_dirs) {
                std::error_code ec;
                const std::filesystem::path candidate = std::filesystem::path(dir) / name;
                if (std::filesystem::is_regular_file(candidate, ec)) {
                    stamp = FileStamp(candidate);
                    break;
                }
            }
            hash.Add(stamp);
            continue;
        }

        std::vector<std::filesystem::path> candidates = {file.parent_path() / name};
        for (const auto& dir : include_dirs)
            candidates.push_back(std::filesystem::path(dir) / name);

        hash.Add(name);
        bool found = false;
        for (const auto& candidate : candidates) {
            std::error_code ec;
            if (!std::filesystem::is_regular_file(candidate, ec))
                continue;
            found = true;
            const std::string resolved = candidate.lexically_normal().generic_string();
            if (!visited.insert(resolved).second) {
                hash.Add(std::string("<seen>"));
                break;
            }
            std::string included;
            ReadFileBytes(candidate, included);
            hash.Add(included);
            HashQuotedIncludes(candidate, included, include_dirs, hash, visited);
            break;
        }
        if (!found)
            hash.Add(std::string("<missing>"));
    }
}

std::string GetEnv(const char* name) {
    const char* value = std::getenv(name);
    return value ? std::string(value) : std::string();
}

std::string ShaderCacheMode() {
    std::string mode = GetEnv("CHRONO_SENSOR_SHADER_CACHE");
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return mode;
}

bool ShaderCacheEnabled() {
    const std::string mode = ShaderCacheMode();
    return !(mode == "0" || mode == "off" || mode == "false");
}

std::mutex shader_cache_mutex;
std::unordered_map<std::string, std::string> shader_memory_cache;

std::atomic<unsigned long long> stat_memory_hits{0};
std::atomic<unsigned long long> stat_disk_hits{0};
std::atomic<unsigned long long> stat_compiles{0};
std::atomic<unsigned long long> stat_programs_created{0};
std::atomic<unsigned long long> stat_programs_destroyed{0};

bool ReadCacheEntry(const std::filesystem::path& path, const std::string& key, std::string& payload) {
    std::string bytes;
    if (!ReadFileBytes(path, bytes))
        return false;
    const size_t header = sizeof(kShaderCacheMagic) + key.size() + 2 * sizeof(uint64_t);
    if (bytes.size() < header || bytes.compare(0, sizeof(kShaderCacheMagic), kShaderCacheMagic, sizeof(kShaderCacheMagic)) != 0 ||
        bytes.compare(sizeof(kShaderCacheMagic), key.size(), key) != 0)
        return false;
    uint64_t size = 0;
    uint64_t checksum = 0;
    std::memcpy(&size, bytes.data() + sizeof(kShaderCacheMagic) + key.size(), sizeof(size));
    std::memcpy(&checksum, bytes.data() + sizeof(kShaderCacheMagic) + key.size() + sizeof(size), sizeof(checksum));
    if (bytes.size() != header + size)
        return false;
    payload = bytes.substr(header);
    return PayloadChecksum(payload) == checksum;
}

// Written to a temporary file and renamed into place, so a concurrent reader sees either no entry or a
// complete one. Failure only costs a recompile next time, so it is not an error.
void WriteCacheEntry(const std::filesystem::path& path, const std::string& key, const std::string& payload) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    std::ostringstream tmp_name;
#ifdef _WIN32
    const long long pid = _getpid();
#else
    const long long pid = getpid();
#endif
    tmp_name << path.filename().string() << ".tmp." << pid << "." << std::this_thread::get_id() << "." << std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path tmp = path.parent_path() / tmp_name.str();
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f.good())
            return;
        const uint64_t size = payload.size();
        const uint64_t checksum = PayloadChecksum(payload);
        f.write(kShaderCacheMagic, sizeof(kShaderCacheMagic));
        f.write(key.data(), key.size());
        f.write(reinterpret_cast<const char*>(&size), sizeof(size));
        f.write(reinterpret_cast<const char*>(&checksum), sizeof(checksum));
        f.write(payload.data(), payload.size());
        if (!f.good()) {
            f.close();
            std::filesystem::remove(tmp, ec);
            return;
        }
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec)
        std::filesystem::remove(tmp, ec);
}

// True for the names this cache writes: "<module>-<32 hex digits>.nvrtc", or that followed by ".tmp.<...>"
// for an interrupted write. Anything else in the cache directory is left alone by Clear.
bool IsShaderCacheEntryName(const std::string& name) {
    size_t ext = name.rfind(".nvrtc.tmp.");
    if (ext == std::string::npos) {
        if (name.size() < 6 || name.compare(name.size() - 6, 6, ".nvrtc") != 0)
            return false;
        ext = name.size() - 6;
    }
    if (ext < 34 || name[ext - 33] != '-')
        return false;
    for (size_t k = ext - 32; k < ext; k++) {
        if (!std::isxdigit(static_cast<unsigned char>(name[k])) || std::isupper(static_cast<unsigned char>(name[k])))
            return false;
    }
    return true;
}

#ifdef USE_CUDA_NVRTC
// Identity of the NVRTC in use, part of every key. nvrtcVersion reports only major.minor, so a patch
// update installed in place (12.4.0 to 12.4.1) would not change it. The path the loaded NVRTC library
// resolves to, with its size and modification time, does (not on Windows, where only the versions are
// keyed and CHRONO_SENSOR_SHADER_CACHE=clear is needed after such an update). The GPU is deliberately
// not part of the key: no -arch is passed to NVRTC, so its output does not depend on the device.
const std::string& NvrtcToolchainId() {
    static const std::string id = [] {
        int major = 0, minor = 0;
        nvrtcVersion(&major, &minor);
        std::string library = "<unknown>";
    #ifndef _WIN32
        Dl_info info;
        if (dladdr(reinterpret_cast<const void*>(&nvrtcVersion), &info) != 0 && info.dli_fname)
            library = FileStamp(info.dli_fname);
    #endif
        std::ostringstream os;
        os << "nvrtc=" << major << "." << minor << " cuda=" << CUDA_VERSION << " optix=" << OPTIX_VERSION << " lib=" << library;
        return os.str();
    }();
    return id;
}
#endif

}  // namespace

namespace shader_cache {

std::string ComputeKey(const std::string& source_file, const std::vector<std::string>& include_dirs, const std::vector<std::string>& options, const std::string& toolchain) {
    std::string source;
    if (!ReadFileBytes(source_file, source))
        throw std::runtime_error("Shader source not found: " + source_file);

    ShaderHash hash;
    hash.Add(std::string(kShaderCacheFormat));
    hash.Add(toolchain);
    hash.Add(source_file);
    hash.Add(source);
    uint64_t n = options.size();
    hash.Add(&n, sizeof(n));
    for (const auto& option : options)
        hash.Add(option);
    std::set<std::string> visited;
    HashQuotedIncludes(std::filesystem::path(source_file), source, include_dirs, hash, visited);
    return hash.Hex();
}

std::string GetDirectory() {
    if (!ShaderCacheEnabled())
        return "";
    std::string dir = GetEnv("CHRONO_SENSOR_SHADER_CACHE_DIR");
    if (!dir.empty())
        return dir;
#ifdef _WIN32
    const std::string base = GetEnv("LOCALAPPDATA");
#else
    std::string base = GetEnv("XDG_CACHE_HOME");
    if (base.empty() && !GetEnv("HOME").empty())
        base = GetEnv("HOME") + "/.cache";
#endif
    if (base.empty())
        return "";
    return (std::filesystem::path(base) / "chrono" / "sensor_shaders").string();
}

void Clear(bool disk) {
    {
        std::lock_guard<std::mutex> lock(shader_cache_mutex);
        shader_memory_cache.clear();
    }
    if (!disk)
        return;
    const std::string dir = GetDirectory();
    std::error_code ec;
    if (dir.empty() || !std::filesystem::is_directory(dir, ec))
        return;
    // Collect first, then remove; every filesystem call takes an error_code, so nothing here throws.
    std::vector<std::filesystem::path> entries;
    for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (IsShaderCacheEntryName(it->path().filename().string()))
            entries.push_back(it->path());
    }
    for (const auto& entry : entries)
        std::filesystem::remove(entry, ec);
}

Stats GetStats() {
    Stats stats;
    stats.memory_hits = stat_memory_hits;
    stats.disk_hits = stat_disk_hits;
    stats.compiles = stat_compiles;
    stats.programs_created = stat_programs_created;
    stats.programs_destroyed = stat_programs_destroyed;
    return stats;
}

std::string Compile(const std::string& file_name, bool emit_optixir, bool* cache_hit) {
    if (cache_hit)
        *cache_hit = false;
#ifdef USE_CUDA_NVRTC
    #if !CH_OPTIX_EMIT_OPTIXIR
    if (emit_optixir)
        throw std::runtime_error("OptiX-IR output requires NVRTC from CUDA 12.0 or newer");
    #endif

    std::string cuda_file = shader_dir + "/" + file_name + ".cu";
    std::string str;
    if (!ReadFileBytes(cuda_file, str))
        throw std::runtime_error("CUDA file not found for NVRTC: " + cuda_file);

    // include directories and compile flags passed from CMake
    std::vector<std::string> include_dirs;
    const char* nvrtc_include_dirs[] = {CUDA_NVRTC_INCLUDE_LIST};
    int num_dirs = sizeof(nvrtc_include_dirs) / sizeof(nvrtc_include_dirs[0]);
    for (int i = 0; i < num_dirs - 1; i++) {
        include_dirs.push_back(nvrtc_include_dirs[i]);
    }
    std::vector<std::string> options;
    for (const std::string& include_dir : include_dirs) {
        options.push_back("-I" + include_dir);
    }
    const char* nvrtc_flags[] = {CUDA_NVRTC_FLAG_LIST};
    int num_flags = sizeof(nvrtc_flags) / sizeof(nvrtc_flags[0]);
    for (int i = 0; i < num_flags - 1; i++) {
        options.push_back(nvrtc_flags[i]);
    }
    if (emit_optixir) {
        options.push_back("--optix-ir");
    }

    // Look the module up in the in-process cache, then on disk.
    std::string key;
    std::filesystem::path disk_entry;
    if (ShaderCacheEnabled()) {
        static std::once_flag clear_once;
        std::call_once(clear_once, [] {
            if (ShaderCacheMode() == "clear")
                Clear(true);
        });

        key = ComputeKey(cuda_file, include_dirs, options, NvrtcToolchainId());

        {
            std::lock_guard<std::mutex> lock(shader_cache_mutex);
            auto it = shader_memory_cache.find(key);
            if (it != shader_memory_cache.end()) {
                stat_memory_hits++;
                if (cache_hit)
                    *cache_hit = true;
                return it->second;
            }
        }

        const std::string dir = GetDirectory();
        if (!dir.empty()) {
            disk_entry = std::filesystem::path(dir) / (file_name + "-" + key + ".nvrtc");
            std::string payload;
            if (ReadCacheEntry(disk_entry, key, payload)) {
                stat_disk_hits++;
                if (cache_hit)
                    *cache_hit = true;
                std::lock_guard<std::mutex> lock(shader_cache_mutex);
                shader_memory_cache[key] = payload;
                return payload;
            }
        }
    }

    // Compile with NVRTC. The program is destroyed on every path, including when compilation throws.
    nvrtcProgram nvrtc_program;
    NVRTC_ERROR_CHECK(nvrtcCreateProgram(&nvrtc_program, str.c_str(), cuda_file.c_str(), 0, NULL, NULL));
    stat_programs_created++;
    struct ProgramGuard {
        nvrtcProgram& program;
        ~ProgramGuard() {
            if (nvrtcDestroyProgram(&program) == NVRTC_SUCCESS)
                stat_programs_destroyed++;
        }
    } guard{nvrtc_program};

    std::vector<const char*> nvrtc_compiler_flag_list;
    for (const std::string& option : options) {
        nvrtc_compiler_flag_list.push_back(option.c_str());
    }

    const nvrtcResult compile_result = nvrtcCompileProgram(nvrtc_program, (int)nvrtc_compiler_flag_list.size(), nvrtc_compiler_flag_list.data());
    stat_compiles++;

    std::string nvrt_compilation_log;
    size_t log_length;
    nvrtcGetProgramLogSize(nvrtc_program, &log_length);
    nvrt_compilation_log.resize(log_length);
    if (log_length > 0) {
        NVRTC_ERROR_CHECK(nvrtcGetProgramLog(nvrtc_program, &nvrt_compilation_log[0]));
    }
    if (compile_result != NVRTC_SUCCESS) {
        throw std::runtime_error(std::string("Error: ").append(__FILE__) + " at line " + std::to_string(__LINE__) + "\n" + nvrt_compilation_log);
    }

    // Retrieve the module. OptiX-IR is binary and can contain embedded NULs, which is safe
    // here because optixModuleCreate is passed the size explicitly rather than relying on the
    // terminator. The PTX branch is byte-for-byte the pre-existing behavior.
    std::string shader;
    size_t shader_size = 0;
    if (emit_optixir) {
    #if CH_OPTIX_EMIT_OPTIXIR
        NVRTC_ERROR_CHECK(nvrtcGetOptiXIRSize(nvrtc_program, &shader_size));
        shader.resize(shader_size);
        NVRTC_ERROR_CHECK(nvrtcGetOptiXIR(nvrtc_program, &shader[0]));
    #endif
    } else {
        NVRTC_ERROR_CHECK(nvrtcGetPTXSize(nvrtc_program, &shader_size));
        shader.resize(shader_size);
        NVRTC_ERROR_CHECK(nvrtcGetPTX(nvrtc_program, &shader[0]));
    }

    if (!key.empty()) {
        if (!disk_entry.empty())
            WriteCacheEntry(disk_entry, key, shader);
        std::lock_guard<std::mutex> lock(shader_cache_mutex);
        shader_memory_cache[key] = shader;
    }
    return shader;
#else
    (void)file_name;
    (void)emit_optixir;
    throw std::runtime_error("shader_cache::Compile requires Chrono::Sensor built with NVRTC (CH_USE_SENSOR_NVRTC)");
#endif  // USE_CUDA_NVRTC
}

}  // namespace shader_cache

void GetShaderFromFile(OptixDeviceContext context,
                       OptixModule& module,
                       const std::string& file_name,
                       OptixModuleCompileOptions& module_compile_options,
                       OptixPipelineCompileOptions& pipeline_compile_options) {
#ifdef USE_CUDA_NVRTC
    // Compile the shader with NVRTC (or take it from the shader cache), emitting either OptiX-IR or PTX.
    //
    // Which of the two the driver accepts is not decidable at compile time. The IR is produced
    // by NVRTC, which comes from the CUDA toolkit, but it is consumed by libnvoptix, which comes
    // from the installed display driver, and OptiX refuses IR newer than its own front end. A
    // CUDA 13.3 NVRTC emits NVVM IR 115 while a 580-series driver tops out at 98, so every
    // optixModuleCreate fails with OPTIX_ERROR_INVALID_INPUT and
    //   "minor NvvmIRVersion (115) newer than tool (should be 98)".
    // CUDA_VERSION describes the producer and says nothing about the consumer, so the choice is
    // made by trying rather than by testing it.
    char log[2048];
    size_t sizeof_log = sizeof(log);

    // Decided once per process rather than per module: the first shader that has to fall back
    // records it here and every later shader skips the attempt that is already known to fail.
    // A benign race at worst costs another process-wide-consistent retry, so no lock is taken.
    static bool emit_optixir = (CH_OPTIX_EMIT_OPTIXIR != 0);

    if (emit_optixir) {
        const std::string optixir = shader_cache::Compile(file_name, true);
        log[0] = '\0';
        sizeof_log = sizeof(log);
        const OptixResult result = optixModuleCreate(context, &module_compile_options, &pipeline_compile_options,
                                                     optixir.c_str(), optixir.size(), log, &sizeof_log, &module);
        if (result == OPTIX_SUCCESS)
            return;

        emit_optixir = false;
        std::cerr << "Chrono::Sensor: this driver's OptiX rejected the OptiX-IR produced by the installed CUDA "
                     "toolkit (" << optixGetErrorName(result) << "); falling back to PTX for all shaders.\n"
                  << log << std::endl;
    }

    const std::string ptx = shader_cache::Compile(file_name, false);
    sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreate(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                        ptx.size(), log, &sizeof_log, &module));

#else
    std::string ptx_file = shader_dir + "/" + file_name + ".ptx";
    std::string ptx;
    std::ifstream f(ptx_file);
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        ptx = source_buffer.str();
    } else {
        throw std::runtime_error("PTX file not found: " + ptx_file);
    }

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreate(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                        ptx.size(), log, &sizeof_log, &module));
#endif  // USE_CUDA_NVRTC
}

void optix_log_callback(unsigned int level, const char* tag, const char* message, void*) {
    std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << "\n";
}

ByteImageData LoadByteImage(const std::string& filename) {
    ByteImageData img_data;
    int w;
    int h;
    int c;
    unsigned char* data = stbi_load(filename.c_str(), &w, &h, &c, 0);

    if (!data) {
        img_data.w = 0;
        img_data.h = 0;
        img_data.c = 0;
        return img_data;  // return if loading failed
    }

    img_data.data = std::vector<unsigned char>(w * h * c);
    img_data.w = w;
    img_data.h = h;
    img_data.c = c;
    memcpy(img_data.data.data(), data, sizeof(unsigned char) * img_data.data.size());

    stbi_image_free(data);

    return img_data;
}

}  // namespace sensor
}  // namespace chrono
