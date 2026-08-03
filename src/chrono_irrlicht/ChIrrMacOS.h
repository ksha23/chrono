// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// macOS-specific helper for the Chrono Irrlicht module.
//
// Irrlicht is consumed as a prebuilt library, so its Cocoa device cannot be patched from Chrono. This helper reaches
// the NSView that Irrlicht renders into and adjusts it from the outside. It is implemented in Objective-C++
// (ChIrrMacOS.mm) and exposed here with a plain C++ interface, so that the rest of the module can remain plain C++.
//
// Only compiled and called on Apple platforms.
// =============================================================================

#ifndef CH_IRR_MACOS_H
#define CH_IRR_MACOS_H

namespace chrono {
namespace irrlicht {

/// Make the OpenGL surface that Irrlicht renders into match the logical (point) size of its window.
///
/// On a Retina display AppKit gives an OpenGL view a backing store backingScaleFactor times larger than the window
/// measured in points, while Irrlicht keeps every dimension it knows about - its screen size, its viewport, its 2D GUI
/// layout and the mouse coordinates it reports - in points. The result is that Irrlicht draws an 800x600 image into
/// the bottom-left corner of a 1600x1200 surface and leaves the rest of the window black.
///
/// Clearing wantsBestResolutionOpenGLSurface shrinks the surface back to the window's point size, so Irrlicht's
/// viewport covers all of it and macOS scales the result up to fill the window. This must be called after the Irrlicht
/// device has been created, while its OpenGL context is current.
///
/// Returns the display's backing scale factor (2.0 on a Retina display, 1.0 otherwise), or 0.0 if the view could not
/// be located, in which case nothing was changed.
double ChIrrMacOSMatchGLSurfaceToWindowSize();

}  // namespace irrlicht
}  // namespace chrono

#endif
