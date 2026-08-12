// Headless cost probe for a scenery manifest.
//
// Deliberately links no visual system at all: this answers "what does ChSceneryModel::Load cost"
// without opening a window or uploading anything to the GPU, so a manifest that would overwhelm a
// machine can be measured safely. Run it under demos_live/mcity/probe_scenery.sh, which adds an
// RSS watchdog.
//
// Build:  see probe_scenery.sh

#include <chrono>
#include <set>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_scenario/ChSceneryModel.h"

using namespace chrono;
using namespace chrono::scenario;

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: probe_scenery <manifest.json> [max_triangles_per_asset]\n");
        return 1;
    }

    ChSystemNSC sys;
    ChSceneryModel scenery;
    ChSceneryOptions opts;
    if (argc > 2)
        opts.max_triangles_per_asset = (unsigned int)std::atoi(argv[2]);

    auto t0 = std::chrono::steady_clock::now();
    bool ok = scenery.Load(sys, argv[1], opts);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (!ok) {
        printf("load failed\n");
        return 1;
    }
    scenery.ReportTo(std::cout);
    printf("  Load() took %.1f s\n", ms / 1000);

    // Count what the visual system would be handed: distinct geometry against total draw items.
    // The gap between them is exactly what a shape-level cache has to collapse.
    size_t shape_instances = 0, distinct_shapes = 0, distinct_meshes = 0, tris_unique = 0, tris_drawn = 0;
    std::set<const void*> seen_shapes, seen_meshes;
    for (const auto& body : scenery.GetBodies()) {
        auto vmodel = body->GetVisualModel();
        if (!vmodel)
            continue;
        for (const auto& si : vmodel->GetShapeInstances()) {
            auto tm = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(si.shape);
            if (!tm)
                continue;
            shape_instances++;
            size_t n = tm->GetMesh() ? tm->GetMesh()->GetNumTriangles() : 0;
            tris_drawn += n;
            if (seen_shapes.insert(tm.get()).second)
                distinct_shapes++;
            if (tm->GetMesh() && seen_meshes.insert(tm->GetMesh().get()).second) {
                distinct_meshes++;
                tris_unique += n;
            }
        }
    }
    printf("  shape instances (draw items): %zu\n", shape_instances);
    printf("  distinct shape objects:       %zu\n", distinct_shapes);
    printf("  distinct meshes (geometry):   %zu\n", distinct_meshes);
    printf("  triangles, unique:            %zu\n", tris_unique);
    printf("  triangles, as drawn:          %zu\n", tris_drawn);
    return 0;
}
