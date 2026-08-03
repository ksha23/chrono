// render_types.h — backend-agnostic scene description (no GPU, no OS, no Chrono types).
// The core (chrono_scene) fills these; any backend (Metal/Vulkan/…) consumes them.
#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace cr {

// One unique geometry (shared by all instances that reference it). Object space.
struct Geometry {
    std::vector<float>   verts;    // 9 floats / triangle (3 xyz)
    std::vector<float>   normals;  // 9 floats / triangle (3 vertex normals)
    std::vector<float>   tangents; // 9 floats / triangle (object-space tangent, for normal mapping)
    std::vector<float>   uv;       // 6 floats / triangle (3 uv); zeros if none
    std::vector<float>   colors;   // 3 floats / triangle (base albedo)
    std::vector<float>   opacity;  // 1 float / triangle: material opacity d (1 = opaque)
    std::vector<float>   roughness;// 1 float / triangle: material roughness (1 = matte, low = glossy)
    std::vector<float>   metallic;  // 1 float / triangle: material metallic (0 = dielectric, 1 = metal)
    std::vector<int>     texId;    // 1 int / triangle: index into RenderScene.texturePaths, or -1
    std::vector<int>     roughTexId;// 1 int / triangle: roughness map (map_Pr) index, or -1
    std::vector<int>     metalTexId;// 1 int / triangle: metallic  map (map_Pm) index, or -1
    std::vector<int>     opacityTexId;// 1 int / triangle: opacity map (map_d) index, or -1
    std::vector<int>     normalTexId;// 1 int / triangle: normal map (norm/map_Bump) index, or -1
    std::vector<float>   specular;   // 4 floats / triangle: Ks.rgb + use_specular_workflow flag (0/1)
    std::vector<float>   emissive;   // 4 floats / triangle: Ke.rgb + emissive_power
    std::vector<float>   texScale;   // 2 floats / triangle: texture UV scale (default 1,1)
    std::vector<int>     ksTexId;    // 1 int / triangle: specular map (map_Ks) index, or -1
    std::vector<int>     keTexId;    // 1 int / triangle: emissive map (map_Ke) index, or -1
    bool                 dynamic = false;  // deforming (re-extracted + refit every frame)
    int triCount() const { return (int)(verts.size() / 9); }
};

// An instance of a geometry placed in the world.
struct Instance {
    int      geom = 0;        // index into RenderScene.geometries
    float    xform[12];       // object->world 4x3, column-major: col0,col1,col2 (basis), col3 (translation)
    float    rot[9];          // rotation (basis columns) used to transform normals to world
    float    tint[3] = {1,1,1};
    uint32_t mat = 0;         // material class (always 0 = normal material-shaded mesh; reserved)
    uint32_t classId = 0;     // semantic class id (from ChVisualMaterial)
    uint32_t instanceId = 0;  // semantic instance id (from ChVisualMaterial)
    float    vel[3] = {0,0,0};// world linear velocity (for radar Doppler)
};

// Whole scene handed to a backend.
struct RenderScene {
    std::vector<Geometry>    geometries;
    std::vector<Instance>    instances;
    std::vector<std::string> texturePaths;   // texId values index into this list
};

// Camera / look-at (orbit is owned by the backend window; this is the resolved target).
struct Camera { float target[3] = {0,0,0}; float fovDeg = 50.0f; };

}  // namespace cr
