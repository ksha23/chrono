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
// A queryable ground height sampled from the geometry that is actually drawn.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>

#include "chrono_vehicle/terrain/ChMeshHeightField.h"

namespace chrono {
namespace vehicle {

bool ChMeshHeightField::SampleTriangle(int idx, double x, double y, double& z, ChVector3d& n) const {
    const auto& t = m_tris[idx];

    // Barycentric test in the XY plane. A degenerate triangle in plan -- a vertical wall seen
    // edge-on -- has zero area here and is skipped: it has no ground height to report.
    double x1 = t[0].x(), y1 = t[0].y();
    double x2 = t[1].x(), y2 = t[1].y();
    double x3 = t[2].x(), y3 = t[2].y();

    double det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
    if (std::fabs(det) < 1e-12)
        return false;

    double l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det;
    double l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det;
    double l3 = 1.0 - l1 - l2;

    // A small tolerance keeps a query landing exactly on a shared edge from falling through the
    // crack between two triangles.
    constexpr double kEps = 1e-9;
    if (l1 < -kEps || l2 < -kEps || l3 < -kEps)
        return false;

    z = l1 * t[0].z() + l2 * t[1].z() + l3 * t[2].z();
    ChVector3d nn = Vcross(t[1] - t[0], t[2] - t[0]);
    double len = nn.Length();
    n = (len > 1e-12) ? nn / len : ChVector3d(0, 0, 1);
    if (n.z() < 0)
        n = -n;
    return true;
}

ChMeshHeightField::ChMeshHeightField()
    : m_x0(0), m_y0(0), m_cell(1), m_nx(0), m_ny(0), m_lo(0, 0, 0), m_hi(0, 0, 0), m_ready(false) {}

void ChMeshHeightField::AddTriangle(const ChVector3d& a, const ChVector3d& b, const ChVector3d& c) {
    m_tris.push_back({a, b, c});
}

void ChMeshHeightField::Build(double cell_size) {
    m_ready = false;
    m_cells.clear();
    if (m_tris.empty())
        return;

    constexpr double kBig = std::numeric_limits<double>::max();
    m_lo = ChVector3d(kBig, kBig, kBig);
    m_hi = ChVector3d(-kBig, -kBig, -kBig);
    for (const auto& t : m_tris) {
        for (const auto& v : t) {
            m_lo = ChVector3d(std::min(m_lo.x(), v.x()), std::min(m_lo.y(), v.y()), std::min(m_lo.z(), v.z()));
            m_hi = ChVector3d(std::max(m_hi.x(), v.x()), std::max(m_hi.y(), v.y()), std::max(m_hi.z(), v.z()));
        }
    }

    m_cell = std::max(cell_size, 1e-3);
    m_x0 = m_lo.x();
    m_y0 = m_lo.y();

    // Cap the grid so a large extent with a small cell size cannot allocate unboundedly; the
    // query stays correct either way, only the number of triangles per bucket changes.
    constexpr long long kMaxCells = 4000000;
    while (true) {
        long long nx = (long long)std::floor((m_hi.x() - m_x0) / m_cell) + 1;
        long long ny = (long long)std::floor((m_hi.y() - m_y0) / m_cell) + 1;
        if (nx * ny <= kMaxCells) {
            m_nx = (int)nx;
            m_ny = (int)ny;
            break;
        }
        m_cell *= 2;
    }

    m_cells.resize((size_t)m_nx * m_ny);
    for (size_t i = 0; i < m_tris.size(); i++) {
        const auto& t = m_tris[i];
        double xmin = std::min({t[0].x(), t[1].x(), t[2].x()});
        double xmax = std::max({t[0].x(), t[1].x(), t[2].x()});
        double ymin = std::min({t[0].y(), t[1].y(), t[2].y()});
        double ymax = std::max({t[0].y(), t[1].y(), t[2].y()});

        int ix0 = std::max(0, (int)std::floor((xmin - m_x0) / m_cell));
        int ix1 = std::min(m_nx - 1, (int)std::floor((xmax - m_x0) / m_cell));
        int iy0 = std::max(0, (int)std::floor((ymin - m_y0) / m_cell));
        int iy1 = std::min(m_ny - 1, (int)std::floor((ymax - m_y0) / m_cell));

        for (int iy = iy0; iy <= iy1; iy++)
            for (int ix = ix0; ix <= ix1; ix++)
                m_cells[CellIndex(ix, iy)].push_back((int)i);
    }

    m_ready = true;
}

bool ChMeshHeightField::HeightBelow(double x, double y, double z_ref, double& z,
                                    ChVector3d* normal, double tolerance) const {
    if (!m_ready)
        return false;

    int ix = (int)std::floor((x - m_x0) / m_cell);
    int iy = (int)std::floor((y - m_y0) / m_cell);
    if (ix < 0 || iy < 0 || ix >= m_nx || iy >= m_ny)
        return false;

    bool below = false, above = false;
    double best_below = 0, best_above = 0;
    ChVector3d n_below(0, 0, 1), n_above(0, 0, 1);

    for (int idx : m_cells[CellIndex(ix, iy)]) {
        double zt;
        ChVector3d nt;
        if (!SampleTriangle(idx, x, y, zt, nt))
            continue;
        if (zt <= z_ref + tolerance) {
            if (!below || zt > best_below) {
                below = true;
                best_below = zt;
                n_below = nt;
            }
        } else {
            if (!above || zt < best_above) {
                above = true;
                best_above = zt;
                n_above = nt;
            }
        }
    }

    if (below) {
        z = best_below;
        if (normal)
            *normal = n_below;
        return true;
    }
    if (above) {
        z = best_above;
        if (normal)
            *normal = n_above;
        return true;
    }
    return false;
}

bool ChMeshHeightField::Height(double x, double y, double& z, ChVector3d* normal) const {
    if (!m_ready)
        return false;

    int ix = (int)std::floor((x - m_x0) / m_cell);
    int iy = (int)std::floor((y - m_y0) / m_cell);
    if (ix < 0 || iy < 0 || ix >= m_nx || iy >= m_ny)
        return false;

    bool found = false;
    double best_z = 0;
    ChVector3d best_n(0, 0, 1);

    for (int idx : m_cells[CellIndex(ix, iy)]) {
        double zt;
        ChVector3d nt;
        if (!SampleTriangle(idx, x, y, zt, nt))
            continue;
        if (!found || zt > best_z) {
            found = true;
            best_z = zt;
            best_n = nt;
        }
    }

    if (!found)
        return false;
    z = best_z;
    if (normal)
        *normal = best_n;
    return true;
}

bool ChMeshHeightField::WriteWavefront(const std::string& path) const {
    std::ofstream f(path);
    if (!f.good())
        return false;

    f << "# ground surface accumulated by ChSceneryModel, in world space\n";
    f << std::fixed << std::setprecision(4);
    for (const auto& t : m_tris)
        for (const auto& v : t)
            f << "v " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
    // Unwelded: one triangle per three consecutive vertices. Collision meshes are built from a
    // BVH over triangles, so sharing vertices buys nothing here and welding 700k of them would
    // cost more than the file it saves.
    for (size_t i = 0; i < m_tris.size(); i++)
        f << "f " << (3 * i + 1) << ' ' << (3 * i + 2) << ' ' << (3 * i + 3) << '\n';
    return f.good();
}

double ChMeshHeightField::GetMeanBucketOccupancy() const {
    if (m_cells.empty())
        return 0;
    size_t total = 0, used = 0;
    for (const auto& c : m_cells) {
        if (!c.empty()) {
            total += c.size();
            used++;
        }
    }
    return used ? (double)total / used : 0;
}

void ChMeshHeightField::GetExtent(ChVector3d& lo, ChVector3d& hi) const {
    lo = m_lo;
    hi = m_hi;
}

}  // end namespace vehicle
}  // end namespace chrono
