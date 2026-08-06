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
//
// Terrain defined by the road surface of an ASAM OpenDRIVE network.
//
// This mirrors the architecture of CRGTerrain: analytic queries answer tire
// contact, while a separately generated triangle mesh carries visualization and
// sensor rendering. Like CRGTerrain, it is meant for tire models that perform
// their own collision detection (ChTMeasy, ChPac89, ChPac02, ChFiala).
//
// Note on fidelity: OpenDRIVE describes road *geometry*, not road *surface*. It
// carries elevation, superelevation and crossfall, but no roughness. For measured
// surface detail use CRGTerrain, or an OpenDRIVE network whose roads reference
// OpenCRG surfaces (ASAM OpenDRIVE section 10.6) -- that pairing is not yet
// implemented here.
//
// =============================================================================

#ifndef CH_OPENDRIVE_TERRAIN_H
#define CH_OPENDRIVE_TERRAIN_H

#include <memory>
#include <string>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChTerrain.h"

#include "chrono_scenario/ChApiScenario.h"
#include "chrono_scenario/ChOpenDriveNetwork.h"

namespace chrono {
namespace scenario {

/// @addtogroup scenario_module
/// @{

/// Rigid terrain whose surface is the road surface of an OpenDRIVE network.
///
/// Height, normal and friction queries are answered analytically from the road manager, so the
/// surface the tires feel is the road geometry itself rather than a tessellation of it. A
/// visualization mesh is generated separately and is free to be coarser.
class ChApiScenario ChOpenDriveTerrain : public vehicle::ChTerrain {
  public:
    /// Construct an OpenDRIVE terrain in the specified system, over the given network.
    /// The network must outlive this terrain and must already be initialized.
    ChOpenDriveTerrain(ChSystem* system, std::shared_ptr<ChOpenDriveNetwork> network);

    ~ChOpenDriveTerrain() {}

    /// Set the coefficient of friction of the road surface (default: 0.8).
    /// OpenDRIVE can specify per-lane friction, but esmini's road manager does not currently
    /// expose it, so this is a single constant unless a FrictionFunctor is installed.
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set the elevation returned for locations off the road network (default: 0).
    /// Queries outside the network cannot be answered from OpenDRIVE geometry.
    void SetOffNetworkHeight(double height) { m_off_network_height = height; }

    /// Set the longitudinal and lateral resolution of the generated visualization mesh.
    /// Defaults: 1 m along the road, 4 samples across each lane.
    void SetMeshResolution(double ds, int lateral_divisions);

    /// Set which lane types are rendered as road surface (default: ChLaneType::ANY_SURFACE).
    ///
    /// Narrowing this to the travel lanes leaves visible holes where a divided road's central
    /// reservation and verges should be. It also updates the network's elevation query mask, so
    /// that what the tires can rest on stays in step with what is drawn.
    void SetSurfaceLaneTypes(int lane_type_mask);

    /// Set a texture applied to the generated road mesh.
    void SetRoadDiffuseTextureFile(const std::string& tex_file, float scale_u = 1, float scale_v = 1);

    /// Generate the visualization mesh for the road network and attach it to the ground body.
    /// Optional: skip this for headless runs that need only the contact surface.
    void CreateVisualizationMesh();

    /// Generate painted lane marking geometry for the network and attach it to the ground body.
    ///
    /// Markings are read from the OpenDRIVE <roadMark> elements, so solid, broken, doubled and
    /// colored lines all follow the file rather than a guessed convention. Each marking is laid
    /// along its lane's outer border, lifted slightly clear of the road surface.
    ///
    /// Call after CreateVisualizationMesh.
    void CreateLaneMarkings();

    /// Set the longitudinal sampling resolution of the marking geometry (default: 0.5 m).
    /// Finer than the road mesh, because a marking on a curve reads as faceted much sooner than
    /// the road surface under it does.
    void SetLaneMarkingResolution(double ds);

    /// Set the dash pattern used for broken markings whose OpenDRIVE entry omits one.
    /// Defaults to 3 m painted, 9 m clear.
    void SetDefaultDashPattern(double dash_length, double dash_space);

    /// Set how far above the road surface markings are drawn (default: 0.02 m).
    /// Only used when the OpenDRIVE entry declares no height of its own. Markings must sit clear
    /// of the road or they z-fight with it.
    void SetLaneMarkingLift(double lift) { m_marking_lift = lift; }

    /// Get the generated road mesh (null until CreateVisualizationMesh is called).
    std::shared_ptr<ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Get the generated lane marking mesh (null until CreateLaneMarkings is called).
    std::shared_ptr<ChTriangleMeshConnected> GetLaneMarkingMesh() const { return m_marking_mesh; }

    /// Get the ground body carrying the visualization assets.
    std::shared_ptr<ChBody> GetGround() const { return m_ground; }

    /// Get the underlying road network.
    std::shared_ptr<ChOpenDriveNetwork> GetNetwork() const { return m_network; }

    /// Export the road mesh to a Wavefront .obj file.
    void ExportMeshWavefront(const std::string& out_dir);

    // --- ChTerrain interface ---------------------------------------------------------------

    /// Get the road surface elevation below the specified location.
    virtual double GetHeight(const ChVector3d& loc) const override;

    /// Get the road surface point below the specified location.
    virtual ChVector3d GetPoint(const ChVector3d& loc) const override;

    /// Get the road surface normal at the point below the specified location.
    virtual ChVector3d GetNormal(const ChVector3d& loc) const override;

    /// Get the coefficient of friction at the point below the specified location.
    /// Defers to the user-provided ChTerrain::FrictionFunctor if one was set.
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override;

  private:
    std::shared_ptr<ChOpenDriveNetwork> m_network;  ///< road network backing all queries
    std::shared_ptr<ChBody> m_ground;               ///< ground body carrying visualization assets
    std::shared_ptr<ChTriangleMeshConnected> m_mesh;  ///< generated visualization mesh

    float m_friction;            ///< constant contact coefficient of friction
    double m_off_network_height;  ///< elevation reported off the network

    double m_mesh_ds;           ///< longitudinal mesh resolution
    int m_mesh_lateral_divs;    ///< lateral mesh samples per lane
    int m_surface_lane_types;   ///< lane types rendered as road surface

    std::string m_texture_file;  ///< optional road texture
    float m_texture_scale_u;
    float m_texture_scale_v;

    std::shared_ptr<ChTriangleMeshConnected> m_marking_mesh;  ///< generated lane marking mesh
    double m_marking_ds;           ///< longitudinal marking resolution
    double m_marking_lift;         ///< default height above the road surface
    double m_default_dash_length;  ///< dash length when OpenDRIVE omits one
    double m_default_dash_space;   ///< dash gap when OpenDRIVE omits one

    /// Append a ribbon of the given width along `centers` into the marking mesh, dashing it if
    /// `dash_length` is positive. Returns the number of quads emitted.
    int AppendMarkingRibbon(const std::vector<ChVector3d>& centers,
                            double width,
                            double lift,
                            double dash_length,
                            double dash_space);
};

/// @} scenario_module

}  // end namespace scenario
}  // end namespace chrono

#endif
