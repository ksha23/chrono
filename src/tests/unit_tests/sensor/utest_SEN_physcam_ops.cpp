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
// Unit tests for the physics-based camera CUDA filter operations (cuda/phys_cam_ops.cu).
//
// 1. Every per-channel (R, G, B) parameter array reaches the right channel of the right kernel.
//    Each operation is run on a known half4 image and compared with a host evaluation of the same
//    formula, using distinct values per channel so a swapped or dropped channel is caught.
//    The operations are launched on a user stream and read back through that same stream only,
//    so the results also confirm the operations are correctly ordered on the stream they are given.
//
// 2. Repeated calls do not grow device memory. cuda_phys_cam_noise once allocated three small
//    device arrays per call and freed only two, leaking one allocation per rendered frame.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include <cuda.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>

#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"

using namespace chrono::sensor;

namespace {

const unsigned int kW = 32;
const unsigned int kH = 16;
const unsigned int kPixels = kW * kH;

// Half precision carries 11 significant bits, so allow a few ULPs relative to the value.
double HalfTol(double v) {
    return 4e-3 * std::max(1.0, std::abs(v));
}

// Host-side staging of an RGBA half4 image.
struct HostImage {
    std::vector<__half> px;
    HostImage() : px(4 * kPixels) {}
    float Get(unsigned int i, int ch) const { return __half2float(px[4 * i + ch]); }
    void Set(unsigned int i, int ch, float v) { px[4 * i + ch] = __float2half(v); }
};

// Smoothly varying, strictly positive test image with a distinct value per channel.
HostImage MakeInput(float alpha) {
    HostImage img;
    for (unsigned int i = 0; i < kPixels; ++i) {
        const float s = 0.25f + 0.75f * (float)i / (float)(kPixels - 1);
        img.Set(i, 0, 1.0f * s);
        img.Set(i, 1, 0.8f * s + 0.1f);
        img.Set(i, 2, 0.6f * s + 0.2f);
        img.Set(i, 3, alpha);
    }
    return img;
}

class PhysCamOps : public ::testing::Test {
  protected:
    void SetUp() override {
        ASSERT_EQ(cudaStreamCreate(&m_stream), cudaSuccess);
        ASSERT_EQ(cudaMalloc(&m_buf_a, sizeof(__half) * 4 * kPixels), cudaSuccess);
        ASSERT_EQ(cudaMalloc(&m_buf_b, sizeof(__half) * 4 * kPixels), cudaSuccess);
        ASSERT_EQ(cudaMalloc(&m_rng_shot, sizeof(curandState_t) * kPixels), cudaSuccess);
        ASSERT_EQ(cudaMalloc(&m_rng_fpn, sizeof(curandState_t) * kPixels), cudaSuccess);
        init_cuda_rng(1234ull, m_rng_shot, kPixels);
        init_cuda_rng(5678ull, m_rng_fpn, kPixels);
        ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    }
    void TearDown() override {
        cudaFree(m_buf_a);
        cudaFree(m_buf_b);
        cudaFree(m_rng_shot);
        cudaFree(m_rng_fpn);
        cudaStreamDestroy(m_stream);
    }

    // Upload and download go through the test stream only (no device-wide synchronization), so a
    // result read here is only correct if the operation was ordered on that stream.
    void Upload(const HostImage& img, void* dst) { ASSERT_EQ(cudaMemcpyAsync(dst, img.px.data(), sizeof(__half) * 4 * kPixels, cudaMemcpyHostToDevice, m_stream), cudaSuccess); }
    void Download(const void* src, HostImage& img) {
        ASSERT_EQ(cudaMemcpyAsync(img.px.data(), src, sizeof(__half) * 4 * kPixels, cudaMemcpyDeviceToHost, m_stream), cudaSuccess);
        ASSERT_EQ(cudaStreamSynchronize(m_stream), cudaSuccess);
        ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    }

    CUstream m_stream = nullptr;
    void* m_buf_a = nullptr;
    void* m_buf_b = nullptr;
    curandState_t* m_rng_shot = nullptr;
    curandState_t* m_rng_fpn = nullptr;
};

// Channel-distinct parameter arrays shared by the cases below.
float kQEs[3] = {0.2f, 0.5f, 0.9f};
float kGains[3] = {0.5f, 1.0f, 2.0f};
float kBiases[3] = {0.1f, 0.2f, 0.3f};
float kDark[3] = {1.0f, 2.0f, 3.0f};

}  // namespace

TEST_F(PhysCamOps, AggregatorAppliesPerChannelQE) {
    const HostImage in = MakeInput(0.7f);
    HostImage out;
    ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
    // N = 2, t = 0.5, C = 1, P = 3, G = 4: scale = G * P / N / N * C * C * t = 1.5
    cuda_phys_cam_aggregator(m_buf_a, kW, kH, 2.f, 0.5f, 1.f, 3.f, kQEs, 4.f, m_stream);
    ASSERT_NO_FATAL_FAILURE(Download(m_buf_a, out));
    for (unsigned int i = 0; i < kPixels; ++i) {
        for (int ch = 0; ch < 3; ++ch) {
            const double ref = in.Get(i, ch) * 1.5 * kQEs[ch];
            ASSERT_NEAR(out.Get(i, ch), ref, HalfTol(ref)) << "pixel " << i << " channel " << ch;
        }
        ASSERT_EQ(out.Get(i, 3), in.Get(i, 3)) << "alpha must be untouched, pixel " << i;
    }
}

TEST_F(PhysCamOps, ExpsrToDVAppliesPerChannelGainsAndBiases) {
    const HostImage in = MakeInput(0.7f);
    const float ISO = 4.f;
    const float gamma = 1.5f;
    for (int crf = 0; crf < 3; ++crf) {
        SCOPED_TRACE("crf_type " + std::to_string(crf));
        HostImage out;
        ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
        cuda_phys_cam_expsr2dv(m_buf_a, m_buf_b, kW, kH, ISO, kGains, kBiases, gamma, crf, m_stream);
        ASSERT_NO_FATAL_FAILURE(Download(m_buf_b, out));
        for (unsigned int i = 0; i < kPixels; ++i) {
            for (int ch = 0; ch < 3; ++ch) {
                const double e = in.Get(i, ch);
                double ref;
                if (crf == 0)
                    ref = kGains[ch] * std::pow(std::log2(ISO * e), gamma) + kBiases[ch];
                else if (crf == 1)
                    ref = 1.0 / (1.0 + std::exp(-(kGains[ch] * ISO * e + kBiases[ch])));
                else
                    ref = kGains[ch] * ISO * e + kBiases[ch];
                ASSERT_NEAR(out.Get(i, ch), ref, HalfTol(ref)) << "pixel " << i << " channel " << ch;
            }
            // sigmoid writes alpha = 1, the other two copy it through
            ASSERT_EQ(out.Get(i, 3), crf == 1 ? 1.0f : in.Get(i, 3)) << "pixel " << i;
        }
    }
}

TEST_F(PhysCamOps, NoiseAppliesPerChannelDarkCurrentGainAndReadSigma) {
    const HostImage in = MakeInput(0.7f);
    const float t = 0.5f;

    // With both noise terms zero the operation is deterministic: E + D * t, per channel.
    float zero[3] = {0.f, 0.f, 0.f};
    HostImage mean;
    ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
    cuda_phys_cam_noise(m_buf_a, kW, kH, t, kDark, zero, zero, m_rng_shot, m_rng_fpn, m_stream);
    ASSERT_NO_FATAL_FAILURE(Download(m_buf_a, mean));
    for (unsigned int i = 0; i < kPixels; ++i)
        for (int ch = 0; ch < 3; ++ch) {
            const double ref = in.Get(i, ch) + (double)kDark[ch] * t;
            ASSERT_NEAR(mean.Get(i, ch), ref, HalfTol(ref)) << "pixel " << i << " channel " << ch;
        }

    // A noise term enabled on exactly one channel must perturb that channel and no other.
    auto check_only_channel = [&](const float* gains, const float* sigmas, int noisy_ch) {
        HostImage out;
        ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
        cuda_phys_cam_noise(m_buf_a, kW, kH, t, kDark, const_cast<float*>(gains), const_cast<float*>(sigmas), m_rng_shot, m_rng_fpn, m_stream);
        ASSERT_NO_FATAL_FAILURE(Download(m_buf_a, out));
        unsigned int changed[3] = {0, 0, 0};
        for (unsigned int i = 0; i < kPixels; ++i)
            for (int ch = 0; ch < 3; ++ch)
                if (out.Get(i, ch) != mean.Get(i, ch))
                    changed[ch]++;
        for (int ch = 0; ch < 3; ++ch) {
            if (ch == noisy_ch)
                EXPECT_GT(changed[ch], kPixels / 2) << "channel " << ch << " should carry the noise";
            else
                EXPECT_EQ(changed[ch], 0u) << "channel " << ch << " should be noise free";
        }
    };
    float gains_b[3] = {0.f, 0.f, 0.5f};
    float sigma_g[3] = {0.f, 0.5f, 0.f};
    {
        SCOPED_TRACE("shot noise gain on B only");
        check_only_channel(gains_b, zero, 2);
    }
    {
        SCOPED_TRACE("read noise sigma on G only");
        check_only_channel(zero, sigma_g, 1);
    }
}

TEST_F(PhysCamOps, VignettingAndDefocusPassThrough) {
    const HostImage in = MakeInput(0.7f);
    HostImage out;

    // Vignetting: E * (1 - G + G * cos(atan(r / f))^4), with r measured on a sensor of width L.
    const float f = 0.01f, L = 0.02f, G = 0.6f;
    ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
    cuda_phys_cam_vignetting(m_buf_a, kW, kH, f, L, G, m_stream);
    ASSERT_NO_FATAL_FAILURE(Download(m_buf_a, out));
    for (unsigned int i = 0; i < kPixels; ++i) {
        const double x = ((double)(i % kW) - (kW - 1) / 2.0) / kW * L;
        const double y = ((double)(i / kW) - (kH - 1) / 2.0) / kW * L;
        const double fall = 1.0 - G + G * std::pow(std::cos(std::atan(std::sqrt(x * x + y * y) / f)), 4);
        for (int ch = 0; ch < 3; ++ch) {
            const double ref = in.Get(i, ch) * fall;
            ASSERT_NEAR(out.Get(i, ch), ref, HalfTol(ref)) << "pixel " << i << " channel " << ch;
        }
    }

    // Defocus blur: a pixel with no depth (d = 0) is copied through, and alpha is set to 1.
    const HostImage in_rgbd = MakeInput(0.f);
    ASSERT_NO_FATAL_FAILURE(Upload(in_rgbd, m_buf_a));
    cuda_phys_cam_defocus_blur(m_buf_a, m_buf_b, kW, kH, 0.012f, 10.f, 4.f, 3.45e-6f, 10.f, 0.f, m_stream);
    ASSERT_NO_FATAL_FAILURE(Download(m_buf_b, out));
    for (unsigned int i = 0; i < kPixels; ++i) {
        for (int ch = 0; ch < 3; ++ch)
            ASSERT_EQ(out.Get(i, ch), in_rgbd.Get(i, ch)) << "pixel " << i << " channel " << ch;
        ASSERT_EQ(out.Get(i, 3), 1.0f) << "pixel " << i;
    }
}

TEST_F(PhysCamOps, RepeatedFramesDoNotGrowDeviceMemory) {
    // One "frame" runs every phys-cam operation once, as the full filter chain does.
    const HostImage in = MakeInput(0.f);
    float zero[3] = {0.f, 0.f, 0.f};
    auto frame = [&]() {
        cuda_phys_cam_defocus_blur(m_buf_a, m_buf_b, kW, kH, 0.012f, 10.f, 4.f, 3.45e-6f, 10.f, 0.f, m_stream);
        cuda_phys_cam_vignetting(m_buf_b, kW, kH, 0.01f, 0.02f, 0.6f, m_stream);
        cuda_phys_cam_aggregator(m_buf_b, kW, kH, 2.f, 0.5f, 1.f, 3.f, kQEs, 4.f, m_stream);
        cuda_phys_cam_noise(m_buf_b, kW, kH, 0.5f, kDark, zero, zero, m_rng_shot, m_rng_fpn, m_stream);
        cuda_phys_cam_expsr2dv(m_buf_b, m_buf_a, kW, kH, 1.f, kGains, kBiases, 1.f, 2, m_stream);
    };

    ASSERT_NO_FATAL_FAILURE(Upload(in, m_buf_a));
    for (int i = 0; i < 100; ++i)  // warm up: first-use allocations inside the CUDA runtime
        frame();
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);

    // cudaMemGetInfo reports device-wide free memory, so another process allocating during the
    // measurement could make a single sample look like growth. A per-frame leak shows up on every
    // attempt; unrelated activity would have to coincide with all of them.
    // Sizing: the leaked 12-byte arrays are sub-allocated from 2 MiB device pages, so free memory
    // drops in 2 MiB steps. On an RTX 5070 Ti the defect cost 4 MiB per 10000 frames; 40000 frames
    // puts it well clear of the 4 MiB allowance, which a leak-free build does not approach.
    const int kFrames = 40000;
    const double kAllowedBytes = 4.0 * 1024 * 1024;
    double best = 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        size_t free0 = 0, free1 = 0, total = 0;
        ASSERT_EQ(cudaMemGetInfo(&free0, &total), cudaSuccess);
        for (int i = 0; i < kFrames; ++i)
            frame();
        ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
        ASSERT_EQ(cudaGetLastError(), cudaSuccess);
        ASSERT_EQ(cudaMemGetInfo(&free1, &total), cudaSuccess);
        const double grown = (double)free0 - (double)free1;
        std::cout << "  attempt " << attempt << ": device memory grew by " << grown / 1024.0 << " KiB over " << kFrames << " frames" << std::endl;
        best = (attempt == 0) ? grown : std::min(best, grown);
        if (best <= kAllowedBytes)
            break;
    }
    EXPECT_LE(best, kAllowedBytes) << "device memory grows with every phys-cam frame (about " << best / kFrames << " bytes per frame); an allocation is not being freed";
}
