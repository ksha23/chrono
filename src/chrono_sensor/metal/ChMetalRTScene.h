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
// Authors: (Metal RT backend)
// =============================================================================
// Backend-neutral scene staging for the Metal RT engine. Wraps the ChSystem ->
// RenderScene extraction (cr::ChScene) and exposes the flat RenderScene the
// Metal renderer consumes.
// =============================================================================

#ifndef CH_METAL_RT_SCENE_H
#define CH_METAL_RT_SCENE_H

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/metal/ChMetalRenderTypes.h"

namespace cr {
class ChScene;
}

namespace chrono {
namespace sensor {

/// A light in the Metal render scene (mirrors what the OptiX ChScene stores).
struct MetalSceneLight {
    ChVector3f pos;    ///< world position (point/spot) or direction (directional)
    float range;       ///< point/spot falloff range (0 = none)
    ChColor color;     ///< color * intensity
    int type;          ///< 0 = point, 1 = directional, 2 = spot
    ChVector3f dir{0,0,-1};  ///< spot axis
    float cosOuter = -1.f;   ///< spot: cos(outer half-angle)
    float cosInner = -1.f;   ///< spot: cos(inner half-angle)
};

class CH_SENSOR_API ChMetalRTScene {
  public:
    ChMetalRTScene();
    ~ChMetalRTScene();

    /// Build (first call / topology change) or refresh (per frame) the render scene.
    void SyncFromSystem(ChSystem* sys);

    // --- lighting (set once; used by the Metal renderer each frame) ---
    void SetAmbientLight(const ChColor& c) { m_ambient = c; }
    ChColor GetAmbientLight() const { return m_ambient; }

    /// HDR equirectangular environment map (Radiance .hdr) for sky + reflections.
    void SetEnvMap(const std::string& path) { m_env_tex = path; }
    const std::string& GetEnvMap() const { return m_env_tex; }
    /// Environment map as an image-based light: same map, scaled radiance (OptiX AddEnvironmentLight).
    /// It lights surfaces via GI bounces (in GI mode) and sets the sky/reflection radiance scale.
    void AddEnvironmentLight(const std::string& path, float intensity_scale = 1.f) { m_env_tex = path; m_env_intensity = intensity_scale; }
    void SetEnvIntensity(float s) { m_env_intensity = s; }
    float GetEnvIntensity() const { return m_env_intensity; }
    void AddPointLight(const ChVector3f& pos, const ChColor& color, float range) {
        m_lights.push_back({pos, range, color, 0});
    }
    void AddDirectionalLight(const ChVector3f& dir, const ChColor& color) {
        m_lights.push_back({dir, 0.f, color, 1});
    }
    /// Spot light. cone_deg = full outer cone angle; penumbra_deg = soft edge width (0 = hard edge).
    void AddSpotLight(const ChVector3f& pos, const ChVector3f& dir, const ChColor& color, float range,
                      float cone_deg, float penumbra_deg = 5.f) {
        MetalSceneLight L{pos, range, color, 2};
        L.dir = dir.GetNormalized();
        float outer = 0.5f * cone_deg * (float)CH_PI / 180.f;
        float inner = std::max(0.f, outer - penumbra_deg * (float)CH_PI / 180.f);
        L.cosOuter = std::cos(outer); L.cosInner = std::cos(inner);
        m_lights.push_back(L);
    }
    void ClearLights() { m_lights.clear(); }
    const std::vector<MetalSceneLight>& GetLights() const { return m_lights; }

    // --- background (used only when no env map is set) & fog, mirroring OptiX Background/ContextParameters ---
    void SetBackgroundGradient(const ChColor& zenith, const ChColor& horizon) { m_bg_mode = 1; m_bg_zenith = zenith; m_bg_horizon = horizon; }
    void SetBackgroundSolid(const ChColor& c) { m_bg_mode = 2; m_bg_zenith = c; }
    int GetBackgroundMode() const { return m_bg_mode; }
    ChColor GetBgZenith() const { return m_bg_zenith; }
    ChColor GetBgHorizon() const { return m_bg_horizon; }
    void SetFog(const ChColor& color, float scattering) { m_fog_color = color; m_fog_scatter = scattering; }
    ChColor GetFogColor() const { return m_fog_color; }
    float GetFogScattering() const { return m_fog_scatter; }

    // --- physical-camera post effects (applied by the Metal shader) ---
    void SetExposure(float e) { m_exposure = e; }
    void SetVignette(float v) { m_vignette = v; }
    void SetSensorNoise(float sigma) { m_noise_sigma = sigma; }
    void SetDepthOfField(float aperture_radius, float focal_dist) { m_aperture_r = aperture_radius; m_focal_dist = focal_dist; }
    float GetExposure() const { return m_exposure; }
    float GetVignette() const { return m_vignette; }
    float GetSensorNoise() const { return m_noise_sigma; }
    float GetApertureRadius() const { return m_aperture_r; }
    float GetFocalDist() const { return m_focal_dist; }

    const cr::RenderScene& GetRenderScene() const { return m_render_scene; }

    /// True when the geometry set changed and acceleration structures must be rebuilt.
    bool StructureDirty() const { return m_structure_dirty; }
    void ClearStructureDirty() { m_structure_dirty = false; }

    ChSystem* GetSystem() const { return m_system; }

  private:
    ChSystem* m_system = nullptr;
    std::unique_ptr<cr::ChScene> m_builder;
    cr::RenderScene m_render_scene;
    bool m_built = false;
    bool m_structure_dirty = false;
    ChColor m_ambient{0.30f, 0.30f, 0.35f};
    std::vector<MetalSceneLight> m_lights;
    std::string m_env_tex;
    float m_env_intensity = 1.f;
    int m_bg_mode = 2;                        // 1 = gradient, else solid (matches OptiX Background default:
    ChColor m_bg_zenith{0.f, 0.f, 0.f};       //   SOLID_COLOR black when no env map / background is set)
    ChColor m_bg_horizon{0.f, 0.f, 0.f};
    ChColor m_fog_color{0.6f, 0.7f, 0.8f};
    float m_fog_scatter = 0.f;
    float m_exposure = 1.f, m_vignette = 0.f, m_noise_sigma = 0.f, m_aperture_r = 0.f, m_focal_dist = 10.f;
};

}  // namespace sensor
}  // namespace chrono

#endif
