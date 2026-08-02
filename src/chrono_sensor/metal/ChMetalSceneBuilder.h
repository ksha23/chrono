// chrono_scene.h — backend-agnostic bridge from a Chrono ChSystem to a RenderScene.
// Cross-platform C++: depends only on Chrono + std. No GPU / OS code here.
#pragma once
#include "chrono_sensor/metal/ChMetalRenderTypes.h"
#include <map>
#include <memory>
#include <string>

namespace chrono { class ChSystem; class ChBody; }
namespace chrono { class ChTriangleMeshConnected; }

namespace cr {

class ChScene {
public:
    explicit ChScene(chrono::ChSystem* sys) : sys_(sys) {}

    // optional flat ground plane (opt-in; never auto-added to avoid z-fighting)
    void setGround(bool on, double z, double size, bool checker) { groundOn_=on; groundZ_=z; groundSize_=size; groundChecker_=checker; }

    // true if the number of visual shapes changed (bodies/shapes added or removed)
    bool topologyChanged() const;

    // full (re)build: fills scene.geometries / instances / texturePaths, and records
    // per-instance sources so refresh() can update transforms & deforming meshes.
    void build(RenderScene& scene);

    // per-frame: recompute instance world transforms from body poses and re-extract
    // any deforming (dynamic) geometry in place.
    void refresh(RenderScene& scene);

private:
    struct InstSrc {                              // how to update instance i each frame
        chrono::ChBody* body = nullptr;          // moving body (nullptr = static/world)
        double sf[12];                           // shape-in-body frame (rot cols + pos), fixed
        int geom = -1;                           // geometry index (dynamic ones re-extracted)
        bool dynamic = false;
        std::shared_ptr<chrono::ChTriangleMeshConnected> mesh;  // for dynamic re-extract
    };
    chrono::ChSystem* sys_;
    std::vector<InstSrc> srcs_;
    std::map<std::string,int> geomCache_;         // shared-geometry key -> geometry index
    mutable int lastShapeCount_ = -1;
    bool groundOn_=false; double groundZ_=0, groundSize_=200; bool groundChecker_=true;
    int countShapes() const;
};

}  // namespace cr
