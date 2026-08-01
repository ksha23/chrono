// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alesandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_VISUAL_SHAPE_TRIANGLE_MESH_H
#define CH_VISUAL_SHAPE_TRIANGLE_MESH_H

#include <vector>

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a triangle mesh shape that can be visualized in some way.
/// A ChVisualShapeTriangleMesh can be attached to a physics object.
/// Provides various rendering options (e.g., drawing as wireframe, performing backface culling, etc.) which may not be
/// supported by a particular visualization system.
class ChApi ChVisualShapeTriangleMesh : public ChVisualShape {
  public:
    ChVisualShapeTriangleMesh();
    ChVisualShapeTriangleMesh(std::shared_ptr<ChTriangleMeshConnected> mesh, bool load_materials = true);
    ~ChVisualShapeTriangleMesh() {}

    /// Associate the mesh asset with a triangle mesh geometry.
    /// Optionally, if `load_materials` is set to `true` and if the provided trimesh was loaded from a Wavefront OBJ
    /// file, associated material files are searched for and visualization materials loaded.
    void SetMesh(std::shared_ptr<ChTriangleMeshConnected> mesh, bool load_materials = true);

    /// Get triangle mesh geometry associated to visual shape.
    std::shared_ptr<ChTriangleMeshConnected> GetMesh() const { return trimesh; }

    bool IsWireframe() const { return wireframe; }
    void SetWireframe(bool mw) { wireframe = mw; }

    bool IsBackfaceCull() const { return backface_cull; }
    void SetBackfaceCull(bool mbc) { backface_cull = mbc; }

    const std::string& GetName() const { return name; }
    void SetName(const std::string& mname) { name = mname; }

    const ChVector3d& GetScale() const { return scale; }
    void SetScale(const ChVector3d& mscale) { scale = mscale; }

    bool IsFixedConnectivity() const { return fixed_connectivity; }
    void SetFixedConnectivity() { fixed_connectivity = true; }

    /// Vertices modified by the most recent call to SetModifiedVertices.
    /// This reflects one publication only. A consumer that does not read the shape after every
    /// publication -- a sensor sampling at 30 Hz against 500 Hz physics, say -- will miss the ones in
    /// between and go stale. Such consumers should use GetModifiedVerticesSince instead.
    const std::vector<int>& GetModifiedVertices() const { return modified_vertices; }

    /// Record the vertices modified since the previous call.
    /// Also appends to a bounded change log so that consumers reading at their own rate can pick up
    /// everything that happened while they were not looking.
    void SetModifiedVertices(const std::vector<int>& vertices);

    /// Current version of the change log. Increases by one on every call to SetModifiedVertices.
    /// A consumer records this alongside whatever it uploaded, and passes it back to
    /// GetModifiedVerticesSince to obtain what has changed since.
    uint64_t GetModifiedVerticesVersion() const { return dirty_version; }

    /// Collect the vertices modified since the given version.
    ///
    /// Returns true and fills 'vertices' when a delta is available. Returns false when the caller has
    /// fallen further behind than the log retains, in which case the caller must refresh the whole
    /// mesh -- which is what it would have been doing anyway. Indices may repeat if a vertex was
    /// touched in more than one publication.
    ///
    /// Callers that have never synchronized should pass 0 only if they have not yet uploaded anything;
    /// otherwise pass the version recorded at the time of their last upload.
    bool GetModifiedVerticesSince(uint64_t since_version, std::vector<int>& vertices) const;

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return trimesh->GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChTriangleMeshConnected> trimesh;

    bool wireframe;
    bool backface_cull;

    std::string name;
    ChVector3d scale;

    bool fixed_connectivity;
    std::vector<int> modified_vertices;

    // Bounded change log, so that a consumer reading at its own cadence can recover every vertex
    // modified since it last looked rather than only those from the latest publication.
    //
    // The log is dropped once it grows past a fraction of the mesh, on the grounds that a consumer
    // that far behind is cheaper to refresh wholesale than to patch. Dropping it moves
    // dirty_log_base forward, which is what makes GetModifiedVerticesSince report a miss.
    std::vector<int> dirty_log;         ///< concatenated indices since dirty_log_base
    std::vector<size_t> dirty_log_ends; ///< end offset in dirty_log of each publication since the base
    uint64_t dirty_version = 0;         ///< publications so far
    uint64_t dirty_log_base = 0;        ///< version the log currently starts from
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeTriangleMesh, 0)

}  // end namespace chrono

#endif
