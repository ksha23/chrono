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
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/core/ChTypes.h"

#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_scenario/ChOpenDriveTerrain.h"

namespace chrono {
namespace scenario {

using vehicle::ChWorldFrame;

namespace {
/// Name given to the generated mesh, and used as the .obj basename on export.
const char* kMeshName = "opendrive_road";
}  // namespace

ChOpenDriveTerrain::ChOpenDriveTerrain(ChSystem* system, std::shared_ptr<ChOpenDriveNetwork> network)
    : m_network(network),
      m_friction(0.8f),
      m_off_network_height(0),
      m_mesh_ds(1.0),
      m_mesh_lateral_divs(4),
      m_texture_scale_u(1),
      m_texture_scale_v(1) {
    if (!m_network || !m_network->IsInitialized()) {
        std::cerr << "ChOpenDriveTerrain: the supplied road network is not initialized" << std::endl;
    }

    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetName("opendrive_ground");
    m_ground->SetPos(ChVector3d(0, 0, 0));
    m_ground->SetFixed(true);
    m_ground->EnableCollision(false);
    system->AddBody(m_ground);
}

void ChOpenDriveTerrain::SetMeshResolution(double ds, int lateral_divisions) {
    m_mesh_ds = std::max(ds, 0.01);
    m_mesh_lateral_divs = std::max(lateral_divisions, 1);
}

void ChOpenDriveTerrain::SetRoadDiffuseTextureFile(const std::string& tex_file, float scale_u, float scale_v) {
    m_texture_file = tex_file;
    m_texture_scale_u = scale_u;
    m_texture_scale_v = scale_v;
}

// -----------------------------------------------------------------------------------------------
// ChTerrain interface
// -----------------------------------------------------------------------------------------------

double ChOpenDriveTerrain::GetHeight(const ChVector3d& loc) const {
    double elevation;
    if (!m_network || !m_network->GetElevation(loc, elevation))
        return m_off_network_height;

    return elevation;
}

ChVector3d ChOpenDriveTerrain::GetPoint(const ChVector3d& loc) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    ChVector3d point_ISO(loc_ISO.x(), loc_ISO.y(), GetHeight(loc));
    return ChWorldFrame::FromISO(point_ISO);
}

ChVector3d ChOpenDriveTerrain::GetNormal(const ChVector3d& loc) const {
    // Finite differences of the elevation, as CRGTerrain does. Deriving the normal from the same
    // height field the tires read guarantees the two agree; the road's own pitch/roll are available
    // separately through ChOpenDriveNetwork::GetLaneInfo.
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    const double delta = 0.05;

    ChVector3d front_loc = ChWorldFrame::FromISO(loc_ISO + ChVector3d(delta, 0, 0));
    ChVector3d left_loc = ChWorldFrame::FromISO(loc_ISO + ChVector3d(0, delta, 0));

    // A network has hard boundaries, unlike a CRG strip. If any of the three probes falls off the
    // network the differences are meaningless, so report a vertical normal rather than a wild one.
    double z0, zfront, zleft;
    if (!m_network || !m_network->GetElevation(loc, z0) || !m_network->GetElevation(front_loc, zfront) ||
        !m_network->GetElevation(left_loc, zleft)) {
        return ChWorldFrame::Vertical();
    }

    ChVector3d p0(loc_ISO.x(), loc_ISO.y(), z0);
    ChVector3d pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
    ChVector3d pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);

    ChVector3d normal_ISO = Vcross(pfront - p0, pleft - p0);
    if (normal_ISO.z() <= 0)
        return ChWorldFrame::Vertical();

    ChVector3d normal = ChWorldFrame::FromISO(normal_ISO);
    normal.Normalize();
    return normal;
}

float ChOpenDriveTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

// -----------------------------------------------------------------------------------------------
// Visualization mesh
//
// One triangle strip per (road, lane), sampled every m_mesh_ds along the road and
// m_mesh_lateral_divs times across the lane. Lanes whose width goes to zero (merges, ramp tapers)
// break the strip, so each contiguous run of stations is emitted separately.
// -----------------------------------------------------------------------------------------------

void ChOpenDriveTerrain::CreateVisualizationMesh() {
    if (!m_network || !m_network->IsInitialized()) {
        std::cerr << "ChOpenDriveTerrain::CreateVisualizationMesh(): no road network" << std::endl;
        return;
    }

    m_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    auto& coords = m_mesh->GetCoordsVertices();
    auto& indices = m_mesh->GetIndicesVertices();
    auto& coords_uv = m_mesh->GetCoordsUV();
    auto& indices_uv = m_mesh->GetIndicesUV();

    const int nv = m_mesh_lateral_divs + 1;  // vertices across one lane

    // Emit the quads spanning a contiguous run of stations, given the index of its first vertex.
    auto emit_strip = [&](int first_vertex, int num_stations) {
        for (int i = 0; i < num_stations - 1; i++) {
            int ofs = first_vertex + nv * i;
            for (int j = 0; j < nv - 1; j++) {
                ChVector3i t1(j + ofs, j + nv + ofs, j + 1 + ofs);
                ChVector3i t2(j + 1 + ofs, j + nv + ofs, j + 1 + nv + ofs);
                indices.push_back(t1);
                indices.push_back(t2);
                indices_uv.push_back(t1);
                indices_uv.push_back(t2);
            }
        }
    };

    for (unsigned int road_id : m_network->GetRoadIds()) {
        double length = m_network->GetRoadLength(road_id);
        if (length <= 0)
            continue;

        int num_steps = static_cast<int>(std::ceil(length / m_mesh_ds));

        // Lane sets can change along a road, so collect the union over all stations.
        std::vector<int> lane_ids = m_network->GetLaneIds(road_id, 0.0, ChLaneType::ANY_ROAD);
        for (int i = 1; i <= num_steps; i++) {
            double s = std::min(i * m_mesh_ds, length);
            for (int lane_id : m_network->GetLaneIds(road_id, s, ChLaneType::ANY_ROAD)) {
                if (std::find(lane_ids.begin(), lane_ids.end(), lane_id) == lane_ids.end())
                    lane_ids.push_back(lane_id);
            }
        }

        for (int lane_id : lane_ids) {
            if (lane_id == 0)  // the reference line carries no surface
                continue;

            int strip_first = static_cast<int>(coords.size());
            int strip_stations = 0;

            for (int i = 0; i <= num_steps; i++) {
                double s = std::min(i * m_mesh_ds, length);
                double width = m_network->GetLaneWidth(road_id, lane_id, s);

                if (width <= 1e-6) {
                    // Lane absent here: close the run and begin a new one after the gap.
                    if (strip_stations >= 2)
                        emit_strip(strip_first, strip_stations);
                    strip_first = static_cast<int>(coords.size());
                    strip_stations = 0;
                    continue;
                }

                bool station_ok = true;
                size_t before = coords.size();

                for (int j = 0; j < nv; j++) {
                    // Sweep the lane from its right edge to its left edge.
                    double frac = static_cast<double>(j) / m_mesh_lateral_divs;
                    double offset = -0.5 * width + frac * width;

                    ChLaneCoord lane_pos{road_id, lane_id, s, offset};
                    ChCoordsys<> csys;
                    if (!m_network->LaneToWorld(lane_pos, csys, false)) {
                        station_ok = false;
                        break;
                    }

                    coords.push_back(csys.pos);
                    // UVs in meters, so a texture tiles at a predictable physical size.
                    coords_uv.push_back(ChVector2d(s, offset));
                }

                if (!station_ok) {
                    coords.resize(before);
                    coords_uv.resize(before);
                    if (strip_stations >= 2)
                        emit_strip(strip_first, strip_stations);
                    strip_first = static_cast<int>(coords.size());
                    strip_stations = 0;
                    continue;
                }

                strip_stations++;
            }

            if (strip_stations >= 2)
                emit_strip(strip_first, strip_stations);
        }
    }

    if (coords.empty()) {
        std::cerr << "ChOpenDriveTerrain::CreateVisualizationMesh(): generated an empty mesh" << std::endl;
        return;
    }

    auto vmesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vmesh->SetMesh(m_mesh);
    vmesh->SetName(kMeshName);

    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(ChColor(0.35f, 0.35f, 0.35f));
    if (!m_texture_file.empty()) {
        material->SetDiffuseColor(ChColor(1.0f, 1.0f, 1.0f));
        material->SetKdTexture(m_texture_file);
        material->SetTextureScale(m_texture_scale_u, m_texture_scale_v);
    }
    vmesh->SetMaterial(0, material);

    m_ground->AddVisualShape(vmesh);
}

void ChOpenDriveTerrain::ExportMeshWavefront(const std::string& out_dir) {
    if (!m_mesh) {
        std::cerr << "ChOpenDriveTerrain::ExportMeshWavefront(): no mesh; call CreateVisualizationMesh()"
                  << std::endl;
        return;
    }

    std::vector<ChTriangleMeshConnected> meshes = {*m_mesh};
    ChTriangleMeshConnected::WriteWavefront(out_dir + "/" + kMeshName + ".obj", meshes);
}

}  // end namespace scenario
}  // end namespace chrono
