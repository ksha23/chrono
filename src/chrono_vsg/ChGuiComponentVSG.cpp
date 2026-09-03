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
// Radu Serban
// =============================================================================

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>

#include "chrono_vsg/ChGuiComponentVSG.h"

namespace chrono {
namespace vsg3d {

ChGuiComponentVSG::ChGuiComponentVSG() : m_visible(true), m_vsys(nullptr), m_placement(Placement::AUTO), m_layout_pos(0.0f, 0.0f), m_layout_size(0.0f, 0.0f) {}

bool ChGuiComponentVSG::BeginWindow(const char* name, bool* p_open, ImGuiWindowFlags flags) {
    if (m_placement == Placement::AUTO)
        ImGui::SetNextWindowPos(m_layout_pos, ImGuiCond_Always);
    return ImGui::Begin(name, p_open, flags);
}

void ChGuiComponentVSG::EndWindow() {
    // Record the size ImGui gave this window in the current frame. The automatic layout arranges the GUI from
    // these measured sizes, so that it never depends on an assumed panel size (which varies with the font size,
    // the display scale, and the content of the panel itself).
    m_layout_size = ImGui::GetWindowSize();
    ImGui::End();
}

float ChGuiComponentVSG::ScaledSize(float size) {
    static constexpr float reference_font_size = 13.0f;  // size of the default Dear ImGui font
    return size * ImGui::GetFontSize() / reference_font_size;
}

void ChGuiComponentVSG::DrawGauge(float val, float v_min, float v_max) {
    ImGui::PushItemWidth(ScaledSize(150.0f));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
    ImGui::SliderFloat("", &val, v_min, v_max, "%.2f");
    ImGui::PopStyleColor();
    ImGui::PopItemWidth();
}

void TextCentered(const char* text, float offset) {
    auto text_width = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX(offset - text_width / 2);
    ImGui::TextUnformatted(text);
}

void ChGuiComponentVSG::Colorbar(vsg::ref_ptr<vsgImGui::Texture> texture,
                                 const ChVector2d& range,
                                 bool bimodal,
                                 float width,
                                 uint32_t deviceID) {
    float min_val = range[0];
    float max_val = range[1];

    int num_items = 5;
    float height = (width * texture->height) / texture->width;
    float item_width = width / (num_items - 1);

    float offset = ImGui::GetCursorPosX() + item_width / 2;
    ImGui::SetCursorPosX(offset);
    ImGui::Image(texture->id(deviceID), ImVec2(width, height));

    char buffer[100];

    ImGui::PushItemWidth(item_width);
    if (bimodal) {
        // so that the colorbar labels line up with the pressure shader modes in the VSG shader without drift
        // from the zero point - lining them up symmetrically
        const int half = (num_items - 1) / 2;
        float delta_neg = (0 - min_val) / half;
        for (int i = 0; i <= half; i++) {
            double val = min_val + i * delta_neg;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + i * item_width);
            ImGui::SameLine();
        }
        float delta_pos = (max_val - 0) / half;
        for (int step = 1; step <= half; step++) {
            double val = step * delta_pos;
            int column = half + step;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + column * item_width);
            if (column < num_items - 1)
                ImGui::SameLine();
        }
    } else {
        float delta = (max_val - min_val) / (num_items - 1);
        for (int i = 0; i < num_items; i++) {
            double val = min_val + i * delta;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + i * item_width);
            ImGui::SameLine();
        }
    }
    ImGui::PopItemWidth();
}

void ChGuiComponentVSG::HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

}  // namespace vsg3d
}  // namespace chrono
