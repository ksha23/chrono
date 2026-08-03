// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Private helpers shared by the GLFW-based Chrono::Sensor visualization filters.
//
// Chrono::Sensor visualization windows are secondary debug views. They are frequently opened inside an application
// that already owns a graphics toolkit and a run loop -- the Chrono::Vehicle demos, for instance, drive an Irrlicht
// window on the same thread that later runs the sensor filter graph. The guards below make sure that opening and
// drawing into those debug windows never leaves global state belonging to the host toolkit disturbed.
//
// This header is internal to the Chrono::Sensor build; it is not installed and must not be included from a public
// Chrono header.
//
// =============================================================================

#ifndef CHFILTERVISUALIZEGUARDS_H
#define CHFILTERVISUALIZEGUARDS_H

#if defined(USE_SENSOR_GLFW) && defined(__APPLE__)
    // CGLGetCurrentContext / CGLSetCurrentContext are formally deprecated along with the rest of macOS OpenGL, but
    // they remain the only way to observe and restore the context that a non-GLFW toolkit made current.
    #ifndef GL_SILENCE_DEPRECATION
        #define GL_SILENCE_DEPRECATION
    #endif
    #include <OpenGL/OpenGL.h>

    #include <dlfcn.h>
    #include <objc/message.h>
    #include <objc/runtime.h>
#endif

namespace chrono {
namespace sensor {

#if defined(USE_SENSOR_GLFW) && defined(__APPLE__)

namespace detail {

/// Address of AppKit's NSApp global, or nullptr when AppKit is not loaded or no NSApplication exists yet.
/// Deliberately avoids +[NSApplication sharedApplication], which would instantiate an NSApplication as a side effect.
inline id GetRunningNSApp() {
    static id* nsapp = reinterpret_cast<id*>(dlsym(RTLD_DEFAULT, "NSApp"));
    return nsapp ? *nsapp : nullptr;
}

inline id GetNSAppDelegate(id app) {
    using MsgSend = id (*)(id, SEL);
    return reinterpret_cast<MsgSend>(objc_msgSend)(app, sel_registerName("delegate"));
}

inline void SetNSAppDelegate(id app, id delegate) {
    using MsgSend = void (*)(id, SEL, id);
    reinterpret_cast<MsgSend>(objc_msgSend)(app, sel_registerName("setDelegate:"), delegate);
}

}  // namespace detail

/// Restore the NSApplication delegate that was installed when the guard was created.
///
/// A macOS process has exactly one NSApplication and exactly one application delegate. glfwInit() unconditionally
/// installs its own GLFWApplicationDelegate and glfwTerminate() resets the delegate to nil; GLFW offers no hint to opt
/// out. When a Chrono::Sensor visualization window is opened alongside a toolkit that already drives the Cocoa run
/// loop, GLFW silently steals that toolkit's delegate. Irrlicht, for one, then sends -isQuit to GLFW's delegate from
/// CIrrDeviceMacOSX::run() and the application dies with an NSInvalidArgumentException ("unrecognized selector").
///
/// When nothing else has claimed the delegate, the guard does nothing and GLFW keeps its own.
class CocoaAppDelegateGuard {
  public:
    CocoaAppDelegateGuard()
        : m_app(detail::GetRunningNSApp()), m_delegate(m_app ? detail::GetNSAppDelegate(m_app) : nullptr) {}

    ~CocoaAppDelegateGuard() {
        if (!m_app || !m_delegate)
            return;
        if (detail::GetNSAppDelegate(m_app) != m_delegate)
            detail::SetNSAppDelegate(m_app, m_delegate);
    }

    CocoaAppDelegateGuard(const CocoaAppDelegateGuard&) = delete;
    CocoaAppDelegateGuard& operator=(const CocoaAppDelegateGuard&) = delete;

  private:
    id m_app;
    id m_delegate;
};

/// Restore the OpenGL context that was current on this thread when the guard was created.
///
/// The Chrono::Sensor filter graph runs on whichever thread calls ChSensorManager::Update(). With the Metal and Vulkan
/// back ends that is the caller's thread, i.e. usually the same thread that drives the host toolkit's renderer. Making
/// a GLFW window's context current there leaves it current after the filter returns, so the host toolkit's next draw
/// call is issued against a foreign context -- with Irrlicht that means drawing with buffer object names from another
/// context, which segfaults inside the GL driver.
///
/// If no context was current beforehand (the usual standalone Chrono::Sensor case) the guard does nothing, so the
/// GLFW context stays current exactly as before.
class GLContextGuard {
  public:
    GLContextGuard() : m_prev(CGLGetCurrentContext()) {}

    ~GLContextGuard() {
        if (m_prev && CGLGetCurrentContext() != m_prev)
            CGLSetCurrentContext(m_prev);
    }

    GLContextGuard(const GLContextGuard&) = delete;
    GLContextGuard& operator=(const GLContextGuard&) = delete;

  private:
    CGLContextObj m_prev;
};

#else

/// No-op outside macOS: no other platform has a process-wide application delegate to protect.
/// The user-provided destructor keeps -Wunused-variable quiet at the use sites.
struct CocoaAppDelegateGuard {
    ~CocoaAppDelegateGuard() {}
};

/// No-op outside macOS. The same context-leak issue exists in principle on WGL and GLX, but Chrono::Sensor only
/// shares a thread with another renderer on the Metal/Vulkan back ends, and only the macOS path is exercised here.
struct GLContextGuard {
    ~GLContextGuard() {}
};

#endif

}  // namespace sensor
}  // namespace chrono

#endif
