// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CH_GUI_COMPONENT_VSG_H
#define CH_GUI_COMPONENT_VSG_H

#include <vsg/all.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/Texture.h>

#include "chrono/core/ChVector2.h"

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

/// @addtogroup vsg_module
/// @{

/// Base class for a GUI component for the VSG run-time visualization system.
class CH_VSG_API ChGuiComponentVSG {
  public:
    /// Placement policy for the ImGui window of a GUI component.
    enum class Placement {
        AUTO,   ///< the visual system arranges the window (flow layout in the viewport work area)
        CUSTOM  ///< the component positions its own window
    };

    ChGuiComponentVSG();
    virtual ~ChGuiComponentVSG() {}

    /// Allow the GUI component to initialize itself.
    /// This function is called after the main GUI container is created.
    virtual void Initialize() {}

    /// Specify the ImGui elements to be rendered for this GUI component.
    virtual void render(vsg::CommandBuffer& cb) = 0;

    /// Set visibility for this GUI component.
    void SetVisibility(bool visible) { m_visible = visible; }

    /// Toggle GUI visibility for this GUI component.
    void ToggleVisibility() { m_visible = !m_visible; }

    /// Return boolean indicating whether or not this GUI component visible.
    bool IsVisible() const { return m_visible; }

    /// Set the placement policy for the window of this GUI component (default: AUTO).
    /// A component that issues its own ImGui::SetNextWindowPos must use CUSTOM, so that the automatic layout
    /// neither positions it nor reserves screen space for it.
    void SetPlacement(Placement placement) { m_placement = placement; }

    /// Return the placement policy for the window of this GUI component.
    Placement GetPlacement() const { return m_placement; }

    /// Set the screen position at which the automatic layout places the window of this GUI component.
    /// Used by the visual system's layout pass; has no effect for a component with CUSTOM placement.
    void SetLayoutPosition(const ImVec2& pos) { m_layout_pos = pos; }

    /// Return the size of the window of this GUI component, as measured in the last frame it was rendered.
    /// Zero until the component is rendered for the first time.
    const ImVec2& GetLayoutSize() const { return m_layout_size; }

    /// Convert a length expressed for the reference GUI font (the 13 pixel Dear ImGui default, for which the
    /// ImGui style metrics are tuned) into pixels at the font size currently in use.
    /// Use this for widget dimensions instead of literal pixel counts, so that they follow the GUI font size and
    /// the display scale rather than assuming a particular DPI.
    static float ScaledSize(float size);

    /// Utility function to draw a gauge.
    static void DrawGauge(float val, float v_min, float v_max);

    /// Utility function to draw a colorbar (colormap legend).
    static void Colorbar(vsg::ref_ptr<vsgImGui::Texture> texture,
                         const ChVector2d& range,
                         bool bimodal,
                         float width,
                         uint32_t deviceID);

    /// Utility function to display a (?) mark which shows a tooltip when hovered.
    static void HelpMarker(const char* desc);

  protected:
    /// Open the ImGui window of this GUI component.
    /// For a component with AUTO placement, the window is positioned where the visual system's layout pass put
    /// it. Must be paired with a call to EndWindow (which measures the window for the next layout pass).
    bool BeginWindow(const char* name, bool* p_open = nullptr, ImGuiWindowFlags flags = 0);

    /// Close the ImGui window of this GUI component, recording its current size for the next layout pass.
    void EndWindow();

    bool m_visible;
    ChVisualSystemVSG* m_vsys;

  private:
    Placement m_placement;  ///< placement policy for the window of this component
    ImVec2 m_layout_pos;    ///< position assigned by the automatic layout
    ImVec2 m_layout_size;   ///< window size measured in the last rendered frame

    friend class ChVisualSystemVSG;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
