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
      m_surface_lane_types(ChLaneType::ANY_SURFACE),
      m_texture_scale_u(1),
      m_texture_scale_v(1),
      m_marking_ds(0.5),
      m_marking_lift(0.02),
      m_default_dash_length(3.0),
      m_default_dash_space(9.0) {
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

void ChOpenDriveTerrain::SetSurfaceLaneTypes(int lane_type_mask) {
    m_surface_lane_types = lane_type_mask;

    // Keep contact in step with what gets drawn, or a vehicle leaving the travel lanes drops
    // through surface it can see.
    if (m_network)
        m_network->SetElevationLaneTypes(lane_type_mask);
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
    // The drawn geometry wins where it covers the query, so the tire contact patch and the
    // rendered surface are the same surface rather than two estimates of it.
    if (m_ground_field && m_ground_field->IsReady()) {
        ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
        double z;
        // The query point's own height matters: it is a tyre contact patch, and the surface it
        // stands on is the highest one at or below it. Ignoring z would let a tree canopy or a
        // sign gantry overhead answer as "ground".
        if (m_ground_field->HeightBelow(loc_ISO.x(), loc_ISO.y(), loc_ISO.z(), z))
            return z;
    }

    double elevation;
    // No on_surface flag wanted here. A tire momentarily past the lane edge -- on a shoulder, or
    // clipping a corner through a junction -- should stand on the nearest road's surface, not be
    // dropped to a constant. With the network's datum at 277 m, as at Mcity, that constant was a
    // 277 m cliff hidden under ground that renders as continuous and flat.
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
        std::vector<int> lane_ids = m_network->GetLaneIds(road_id, 0.0, m_surface_lane_types);
        for (int i = 1; i <= num_steps; i++) {
            double s = std::min(i * m_mesh_ds, length);
            for (int lane_id : m_network->GetLaneIds(road_id, s, m_surface_lane_types)) {
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

// -----------------------------------------------------------------------------------------------
// Lane markings
//
// One ribbon per marked lane border, laid along the border line and lifted clear of the road.
// The border a marking belongs to is set by OpenDRIVE: a lane's <roadMark> describes its border
// away from the reference line, and the center lane (id 0) describes the road center line.
// -----------------------------------------------------------------------------------------------

void ChOpenDriveTerrain::SetLaneMarkingResolution(double ds) {
    m_marking_ds = std::max(ds, 0.05);
}

void ChOpenDriveTerrain::SetDefaultDashPattern(double dash_length, double dash_space) {
    m_default_dash_length = std::max(dash_length, 0.0);
    m_default_dash_space = std::max(dash_space, 0.0);
}

int ChOpenDriveTerrain::AppendMarkingRibbon(const std::vector<ChVector3d>& centers,
                                            double width,
                                            double lift,
                                            double dash_length,
                                            double dash_space) {
    if (centers.size() < 2)
        return 0;

    auto& coords = m_marking_mesh->GetCoordsVertices();
    auto& indices = m_marking_mesh->GetIndicesVertices();

    const ChVector3d up = ChWorldFrame::Vertical();
    const double half = 0.5 * width;
    const double period = dash_length + dash_space;
    const bool dashed = dash_length > 0 && dash_space > 0;

    double arc = 0;
    int quads = 0;

    for (size_t i = 0; i + 1 < centers.size(); i++) {
        const ChVector3d& p0 = centers[i];
        const ChVector3d& p1 = centers[i + 1];

        ChVector3d seg = p1 - p0;
        double len = seg.Length();
        if (len < 1e-9)
            continue;

        // Keep whole segments: a dash boundary falling mid-segment is rounded to the segment,
        // which at the marking sampling resolution is a sub-decimetre effect.
        if (dashed && std::fmod(arc, period) >= dash_length) {
            arc += len;
            continue;
        }
        arc += len;

        ChVector3d tangent = seg / len;
        ChVector3d lateral = Vcross(up, tangent);
        double lat_len = lateral.Length();
        if (lat_len < 1e-9)
            continue;  // segment is vertical; nothing sensible to lay a ribbon on
        lateral /= lat_len;

        ChVector3d offset = lateral * half;
        ChVector3d rise = up * lift;

        int base = static_cast<int>(coords.size());
        coords.push_back(p0 - offset + rise);
        coords.push_back(p0 + offset + rise);
        coords.push_back(p1 - offset + rise);
        coords.push_back(p1 + offset + rise);

        indices.push_back(ChVector3i(base + 0, base + 2, base + 1));
        indices.push_back(ChVector3i(base + 1, base + 2, base + 3));
        quads++;
    }

    return quads;
}

void ChOpenDriveTerrain::CreateLaneMarkings() {
    if (!m_network || !m_network->IsInitialized()) {
        std::cerr << "ChOpenDriveTerrain::CreateLaneMarkings(): no road network" << std::endl;
        return;
    }

    auto marked = m_network->GetMarkedLanes();
    if (marked.empty()) {
        std::cerr << "ChOpenDriveTerrain::CreateLaneMarkings(): the OpenDRIVE file declares no "
                     "road marks"
                  << std::endl;
        return;
    }

    m_marking_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();

    // Grouped by color so that a single mesh can still carry yellow center lines alongside white
    // lane lines: one mesh per color would be tidier, but Chrono visual shapes take one material
    // per mesh and the overwhelming majority of markings are white.
    ChColor dominant(1.0f, 1.0f, 1.0f);
    int painted = 0;

    for (const auto& key : marked) {
        unsigned int road_id = key.first;
        int lane_id = key.second;

        ChLaneMarkingStyle style = m_network->GetLaneMarkingStyle(road_id, lane_id);
        if (!style.IsPainted())
            continue;

        double length = m_network->GetRoadLength(road_id);
        if (length <= 0)
            continue;

        double lift = style.height > 0 ? style.height : m_marking_lift;
        double dash_length = style.dash_length;
        double dash_space = style.dash_space;
        if (style.IsBroken() && dash_length <= 0) {
            dash_length = m_default_dash_length;
            dash_space = m_default_dash_space;
        }

        // A double marking is drawn as two ribbons straddling the border, one marking width apart.
        std::vector<double> lateral_shifts = style.IsDouble()
                                                 ? std::vector<double>{-style.width, style.width}
                                                 : std::vector<double>{0.0};

        for (double shift : lateral_shifts) {
            std::vector<ChVector3d> centers;
            int num_steps = static_cast<int>(std::ceil(length / m_marking_ds));
            centers.reserve(num_steps + 1);

            for (int i = 0; i <= num_steps; i++) {
                double s = std::min(i * m_marking_ds, length);
                ChCoordsys<> pose;

                if (lane_id == 0) {
                    // The center lane carries no width; its marking sits on the reference line.
                    if (!m_network->RoadToWorld(road_id, s, shift, pose))
                        continue;
                } else {
                    double width = m_network->GetLaneWidth(road_id, lane_id, s);
                    if (width <= 1e-6)
                        continue;

                    // The border away from the reference line: to the right for negative lane
                    // ids, to the left for positive ones. Offsets are measured from lane center
                    // with positive to the left.
                    double border = (lane_id < 0) ? -0.5 * width : 0.5 * width;
                    double sign = (lane_id < 0) ? -1.0 : 1.0;

                    if (!m_network->LaneToWorld({road_id, lane_id, s, border + sign * shift}, pose, false))
                        continue;
                }

                centers.push_back(pose.pos);
            }

            if (AppendMarkingRibbon(centers, style.width, lift, dash_length, dash_space) > 0) {
                painted++;
                dominant = style.color;
            }
        }
    }

    if (m_marking_mesh->GetCoordsVertices().empty()) {
        std::cerr << "ChOpenDriveTerrain::CreateLaneMarkings(): no marking geometry generated"
                  << std::endl;
        m_marking_mesh = nullptr;
        return;
    }

    auto vmesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vmesh->SetMesh(m_marking_mesh);
    vmesh->SetName("opendrive_lane_markings");

    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(dominant);
    // Road paint is retroreflective, which matters for how it reads under headlights at night.
    material->SetSpecularColor(ChColor(0.25f, 0.25f, 0.25f));
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
