
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018, Colin Vanden Heuvel
// All rights reserved.
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Colin Vanden Heuvel
// =============================================================================

#ifndef CH_DEM_GPU_APPLOC_H
#define CH_DEM_GPU_APPLOC_H

#include <climits>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "chrono_dem/ChDemDefines.h"

////#if (__cplusplus >= 201703L)  // C++17 or newer
////template <class T>
////struct gpuallocator {
////  public:
////    std::true_type is_always_equal;
////#else  // C++14 or older
template <class T>
class gpuallocator {
  public:
    typedef T* pointer;
    typedef T& reference;
    typedef const T* const_pointer;
    typedef const T& const_reference;

    template <class U>
    struct rebind {
        typedef typename ::gpuallocator<U> other;
    };

#if (__cplusplus >= 201402L)  // C++14
    std::false_type propagate_on_container_copy_assignment;
    std::false_type propagate_on_container_move_assignment;
    std::false_type propagate_on_container_swap;
#endif
    ////#endif

    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    ////#if (__cplusplus > 201703L)  // newer than (but not including) C++17
    ////    constexpr gpuallocator() noexcept {};
    ////    constexpr gpuallocator(const gpuallocator& other) noexcept {}
    ////
    ////    template <class U>
    ////    constexpr gpuallocator(const gpuallocator<U>& other) noexcept {}
    ////#else  // C++17 or older
    gpuallocator() noexcept {}
    gpuallocator(const gpuallocator& other) noexcept {}

    template <class U>
    gpuallocator(const gpuallocator<U>& other) noexcept {}
    ////#endif

    ////#if (__cplusplus < 201703L)  // before C++17
    pointer address(reference x) const noexcept { return &x; }

    size_type max_size() const noexcept { return ULLONG_MAX / sizeof(T); }

    template <class... Args>
    void construct(T* p, Args&&... args) {
        ::new ((void*)p) T(std::forward<Args>(args)...);
    }
    void destroy(T* p) { p->~T(); }
    ////#endif

    pointer allocate(size_type n, std::allocator<void>::const_pointer hint = 0) {
        void* vptr;
        gpuError err = gpuMallocManaged(&vptr, n * sizeof(T), gpuMemAttachGlobal);
        if (err == gpuErrorMemoryAllocation || err == gpuErrorNotSupported) {
            throw std::bad_alloc();
        }
        return (T*)vptr;
    }

    void deallocate(pointer p, size_type n) {
        if (p) {
            demErrchk(gpuFree(p));
        }
    }

    bool operator==(const gpuallocator& other) const { return true; }
    bool operator!=(const gpuallocator& other) const { return false; }
};

/// Array in device-only memory (gpuMalloc), for data that only kernels read or write.
/// Unlike a std::vector with gpuallocator (managed memory), the host cannot dereference the elements. Host code copies
/// them explicitly with CopyToHost / CopyFromHost / Get. Managed memory that is written by kernels at every step either
/// migrates pages back and forth (CUDA) or, where the device cannot fault on host pages (HIP without XNACK), stays in
/// host memory so that every kernel access crosses the bus.
template <class T>
class gpudevicevector {
  public:
    typedef T value_type;

    gpudevicevector() : m_data(nullptr), m_size(0) {}
    ~gpudevicevector() {
        if (m_data)
            gpuFree(m_data);
    }
    gpudevicevector(const gpudevicevector&) = delete;
    gpudevicevector& operator=(const gpudevicevector&) = delete;

    std::size_t size() const { return m_size; }
    bool empty() const { return m_size == 0; }
    T* data() { return m_data; }
    const T* data() const { return m_data; }

    /// Resize the array, keeping the first min(size(), n) elements and setting new elements to val.
    void resize(std::size_t n, const T& val = T()) {
        if (n == m_size)
            return;
        T* ptr = nullptr;
        if (n > 0) {
            if (gpuMalloc((void**)&ptr, n * sizeof(T)) != gpuSuccess)
                throw std::bad_alloc();
            try {
                std::size_t keep = n < m_size ? n : m_size;
                if (keep > 0)
                    demErrchk(gpuMemcpy(ptr, m_data, keep * sizeof(T), gpuMemcpyDeviceToDevice));
                if (n > keep)
                    Fill(ptr + keep, n - keep, val);
            } catch (...) {
                gpuFree(ptr);  // the array keeps its old contents
                throw;
            }
        }
        if (m_data)
            demErrchk(gpuFree(m_data));
        m_data = ptr;
        m_size = n;
    }

    void clear() { resize(0); }

    void swap(gpudevicevector& other) noexcept {
        std::swap(m_data, other.m_data);
        std::swap(m_size, other.m_size);
    }

    /// Copy count elements starting at offset into host memory.
    void CopyToHost(T* dst, std::size_t offset, std::size_t count) const {
        CheckRange(offset, count);
        if (count > 0)
            demErrchk(gpuMemcpy(dst, m_data + offset, count * sizeof(T), gpuMemcpyDeviceToHost));
    }

    /// Copy the whole array into a host vector (resized to size()).
    template <class Alloc>
    void CopyToHost(std::vector<T, Alloc>& dst) const {
        dst.resize(m_size);
        CopyToHost(dst.data(), 0, m_size);
    }

    /// Copy count elements from host memory into the array, starting at offset.
    void CopyFromHost(const T* src, std::size_t offset, std::size_t count) {
        CheckRange(offset, count);
        if (count > 0)
            demErrchk(gpuMemcpy(m_data + offset, src, count * sizeof(T), gpuMemcpyHostToDevice));
    }

    /// Read one element (bounds checked, like std::vector::at).
    T Get(std::size_t i) const {
        T val;
        CopyToHost(&val, i, 1);
        return val;
    }

  private:
    void CheckRange(std::size_t offset, std::size_t count) const {
        if (offset > m_size || count > m_size - offset)
            throw std::out_of_range("gpudevicevector: index out of range");
    }

    // Set n elements at dst to val: a memset when all bytes of val are equal (zero, NULL_CHDEM_ID, false), else a
    // host-to-device copy through a bounded staging buffer.
    static void Fill(T* dst, std::size_t n, const T& val) {
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&val);
        bool uniform = true;
        for (std::size_t b = 1; b < sizeof(T); b++)
            uniform = uniform && (bytes[b] == bytes[0]);
        if (uniform) {
            demErrchk(gpuMemset(dst, bytes[0], n * sizeof(T)));
            return;
        }
        const std::size_t chunk = n < (1u << 20) ? n : (1u << 20);
        std::vector<T> staging(chunk, val);
        for (std::size_t i = 0; i < n; i += chunk) {
            std::size_t count = (n - i) < chunk ? (n - i) : chunk;
            demErrchk(gpuMemcpy(dst + i, staging.data(), count * sizeof(T), gpuMemcpyHostToDevice));
        }
    }

    T* m_data;
    std::size_t m_size;
};

#endif
