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
// Tests for ChConvexHull2D: the monotone chain method, on the full point set and on the per-column extremes of grid
// patches (as used by SCM contact patches), must give the area and perimeter of an exact reference hull. The reference
// hull is computed by brute force in integer grid coordinates, so it does not depend on the Jarvis method, on the
// input point order, or on floating point round-off.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChVector2.h"
#include "chrono/utils/ChConvexHull.h"

using namespace chrono;
using namespace chrono::utils;

// Grid nodes of a 4-connected patch, stored as integer (i,j) pairs.
using GridPatch = std::vector<std::pair<int, int>>;

static GridPatch Ellipse(int a, int b) {
    GridPatch p;
    for (int i = -a; i <= a; i++)
        for (int j = -b; j <= b; j++)
            if (double(i * i) / (a * a) + double(j * j) / (b * b) <= 1.0)
                p.push_back({i, j});
    return p;
}

static GridPatch Rectangle(int w, int h) {
    GridPatch p;
    for (int i = 0; i < w; i++)
        for (int j = 0; j < h; j++)
            p.push_back({i, j});
    return p;
}

// Ring between two ellipses (hull is set by the outer boundary only).
static GridPatch Annulus(int a, int b) {
    GridPatch p;
    for (int i = -a; i <= a; i++)
        for (int j = -b; j <= b; j++) {
            double r = double(i * i) / (a * a) + double(j * j) / (b * b);
            if (r <= 1.0 && r >= 0.36)
                p.push_back({i, j});
        }
    return p;
}

// Random 4-connected blob grown from the origin.
static GridPatch RandomBlob(int n, unsigned seed) {
    std::mt19937 rng(seed);
    std::set<std::pair<int, int>> in = {{0, 0}};
    GridPatch p = {{0, 0}};
    const int di[4] = {1, -1, 0, 0};
    const int dj[4] = {0, 0, 1, -1};
    while ((int)p.size() < n) {
        auto c = p[rng() % p.size()];
        int k = rng() % 4;
        std::pair<int, int> q = {c.first + di[k], c.second + dj[k]};
        if (in.insert(q).second)
            p.push_back(q);
    }
    return p;
}

// Same reduction as SCMTerrain: lowest and highest node in each grid column.
static std::vector<ChVector2d> ColumnExtremes(const GridPatch& p, double delta, const ChVector2d& offset) {
    int imin = p[0].first, imax = p[0].first;
    for (const auto& ij : p) {
        imin = std::min(imin, ij.first);
        imax = std::max(imax, ij.first);
    }
    std::vector<int> jmin(imax - imin + 1, std::numeric_limits<int>::max());
    std::vector<int> jmax(imax - imin + 1, std::numeric_limits<int>::lowest());
    for (const auto& ij : p) {
        int c = ij.first - imin;
        jmin[c] = std::min(jmin[c], ij.second);
        jmax[c] = std::max(jmax[c], ij.second);
    }
    std::vector<ChVector2d> pts;
    for (int c = 0; c < (int)jmin.size(); c++) {
        if (jmin[c] > jmax[c])
            continue;
        pts.push_back(offset + ChVector2d(delta * (imin + c), delta * jmin[c]));
        if (jmax[c] != jmin[c])
            pts.push_back(offset + ChVector2d(delta * (imin + c), delta * jmax[c]));
    }
    return pts;
}

// Portable shuffle (std::shuffle is implementation-defined; std::mt19937 output is not).
static std::vector<ChVector2d> AllPoints(const GridPatch& p, double delta, const ChVector2d& offset, unsigned seed) {
    std::vector<ChVector2d> pts;
    for (const auto& ij : p)
        pts.push_back(offset + ChVector2d(delta * ij.first, delta * ij.second));
    std::mt19937 rng(seed);
    for (size_t i = pts.size(); i > 1; i--)
        std::swap(pts[i - 1], pts[rng() % i]);  // SCM patch order follows hash-map iteration
    return pts;
}

// Exact reference hull of a grid patch, in integer grid coordinates.
struct RefHull {
    long long area2;      // twice the hull area, in grid units
    double perimeter;     // hull perimeter, in grid units
    size_t num_vertices;  // number of strictly convex hull vertices
};

static long long Cross(const std::pair<int, int>& o, const std::pair<int, int>& a, const std::pair<int, int>& b) {
    return (long long)(a.first - o.first) * (b.second - o.second) - (long long)(a.second - o.second) * (b.first - o.first);
}

// Brute force: (a,b) is a hull edge if every point lies to its left or on the segment [a,b]. On each side of the hull
// only the edge between the two extreme collinear points qualifies, so the edges are exactly the hull sides.
// The candidates are the column extremes; the test separately checks that every patch node lies inside the result,
// which shows that the hull of the column extremes is the hull of the whole patch.
static RefHull ReferenceHull(const GridPatch& patch) {
    std::map<int, std::pair<int, int>> col;  // column -> (jmin, jmax)
    for (const auto& ij : patch) {
        auto it = col.find(ij.first);
        if (it == col.end())
            col[ij.first] = {ij.second, ij.second};
        else
            it->second = {std::min(it->second.first, ij.second), std::max(it->second.second, ij.second)};
    }
    GridPatch c;
    for (const auto& e : col) {
        c.push_back({e.first, e.second.first});
        if (e.second.second != e.second.first)
            c.push_back({e.first, e.second.second});
    }

    RefHull ref = {0, 0.0, 0};
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> edges;
    for (const auto& a : c) {
        for (const auto& b : c) {
            if (a == b)
                continue;
            long long ex = b.first - a.first, ey = b.second - a.second;
            bool is_edge = true;
            for (const auto& q : c) {
                long long cr = Cross(a, b, q);
                long long dt = (q.first - a.first) * ex + (q.second - a.second) * ey;
                if (cr < 0 || (cr == 0 && (dt < 0 || dt > ex * ex + ey * ey))) {
                    is_edge = false;
                    break;
                }
            }
            if (is_edge)
                edges.push_back({a, b});
        }
    }
    for (const auto& e : edges) {
        ref.area2 += (long long)e.first.first * e.second.second - (long long)e.first.second * e.second.first;
        ref.perimeter += std::hypot(double(e.second.first - e.first.first), double(e.second.second - e.first.second));
    }
    ref.num_vertices = edges.size();

    // Every patch node must be inside (or on) the hull of the column extremes.
    for (const auto& q : patch)
        for (const auto& e : edges)
            EXPECT_GE(Cross(e.first, e.second, q), 0);

    return ref;
}

static void Check(const GridPatch& patch, double delta, const ChVector2d& offset) {
    RefHull ref = ReferenceHull(patch);
    ASSERT_GT(ref.area2, 0);
    double A = 0.5 * ref.area2 * delta * delta;
    double P = ref.perimeter * delta;

    auto pts = AllPoints(patch, delta, offset, 7);
    ChConvexHull2D monotone(pts, ChConvexHull2D::MONOTONE);

    auto red = ColumnExtremes(patch, delta, offset);
    ChConvexHull2D reduced(red, ChConvexHull2D::MONOTONE);

    EXPECT_NEAR(monotone.GetArea(), A, 1e-12 * A);
    EXPECT_NEAR(monotone.GetPerimeter(), P, 1e-12 * P);
    EXPECT_NEAR(reduced.GetArea(), A, 1e-12 * A);
    EXPECT_NEAR(reduced.GetPerimeter(), P, 1e-12 * P);

    // The monotone chain hull is closed and contains only the strictly convex vertices.
    const auto& h = reduced.GetHull();
    ASSERT_GE(h.size(), 4u);
    EXPECT_EQ(h.size(), ref.num_vertices + 1);
    EXPECT_EQ(monotone.GetHull().size(), ref.num_vertices + 1);
    EXPECT_EQ(h.front(), h.back());
    for (size_t i = 1; i + 1 < h.size(); i++) {
        ChVector2d e1 = h[i] - h[i - 1];
        ChVector2d e2 = h[i + 1] - h[i];
        EXPECT_GT(e1.x() * e2.y() - e1.y() * e2.x(), 0);  // strict left turn (counterclockwise)
    }
}

static const double kDelta[] = {0.02, 0.005};
static const ChVector2d kOffset[] = {ChVector2d(0, 0), ChVector2d(1.234, -3.21), ChVector2d(-12.5, 7.75)};

TEST(ChConvexHull2D, ellipse) {
    for (double d : kDelta)
        for (const auto& o : kOffset)
            for (int r : {2, 5, 20, 60})
                Check(Ellipse(r, 2 * r), d, o);
}

TEST(ChConvexHull2D, rectangle) {
    for (double d : kDelta)
        for (const auto& o : kOffset)
            for (int r : {2, 7, 40}) {
                Check(Rectangle(r, 4 * r), d, o);

                // Exact values, and only the 4 corners (plus closure) in the hull
                auto red = ColumnExtremes(Rectangle(r, 4 * r), d, o);
                ChConvexHull2D ch(red, ChConvexHull2D::MONOTONE);
                double w = d * (r - 1), h = d * (4 * r - 1);
                EXPECT_NEAR(ch.GetArea(), w * h, 1e-12 * w * h);
                EXPECT_NEAR(ch.GetPerimeter(), 2 * (w + h), 1e-12 * (w + h));
                EXPECT_EQ(ch.GetHull().size(), 5u);
            }
}

TEST(ChConvexHull2D, annulus) {
    for (double d : kDelta)
        for (const auto& o : kOffset)
            for (int r : {6, 25, 50})
                Check(Annulus(r, 2 * r), d, o);
}

TEST(ChConvexHull2D, random) {
    for (double d : kDelta)
        for (const auto& o : kOffset)
            for (unsigned seed = 1; seed <= 20; seed++)
                Check(RandomBlob(10 + 150 * seed, seed), d, o);
}

// A 3x4 node SCM contact patch (grid spacing 0.05) recorded from the M113 SCM model, in its flood-fill order.
// For this point order, the Jarvis method returns area 0.01375 and perimeter 0.4707 (it skips the corner node
// (-109,20)). The exact hull is the 0.10 x 0.15 rectangle.
TEST(ChConvexHull2D, rectangle_patch_order) {
    const GridPatch nodes = {{-110, 20}, {-111, 20}, {-109, 20}, {-110, 21}, {-111, 21}, {-109, 21}, {-110, 22}, {-111, 22}, {-109, 22}, {-110, 23}, {-111, 23}, {-109, 23}};
    const double d = 0.05;
    std::vector<ChVector2d> pts;
    for (const auto& ij : nodes)
        pts.push_back(ChVector2d(d * ij.first, d * ij.second));
    ChConvexHull2D ch(pts, ChConvexHull2D::MONOTONE);
    EXPECT_NEAR(ch.GetArea(), 0.015, 1e-12);
    EXPECT_NEAR(ch.GetPerimeter(), 0.5, 1e-12);

    auto red = ColumnExtremes(nodes, d, ChVector2d(0, 0));
    ChConvexHull2D chr(red, ChConvexHull2D::MONOTONE);
    EXPECT_NEAR(chr.GetArea(), 0.015, 1e-12);
    EXPECT_NEAR(chr.GetPerimeter(), 0.5, 1e-12);
}

// Degenerate patches: area is zero, so SCM does not use the perimeter (1/b is set to 0).
TEST(ChConvexHull2D, degenerate) {
    for (int n : {4, 10}) {
        GridPatch line;
        for (int i = 0; i < n; i++)
            line.push_back({i, 3});
        auto red = ColumnExtremes(line, 0.01, ChVector2d(1, 2));
        ChConvexHull2D ch(red, ChConvexHull2D::MONOTONE);
        EXPECT_NEAR(ch.GetArea(), 0.0, 1e-15);
    }
}
