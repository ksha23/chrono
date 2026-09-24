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
// Test of ChBox volume, gyration, and bounding volumes against hand-computed
// values, for boxes with integer and non-integer half-lengths.
//
// =============================================================================

#include <cmath>

#include "gtest/gtest.h"
#include "chrono/geometry/ChBox.h"

using namespace chrono;

const double ABS_ERR = 1e-12;

static void CheckBox(const ChVector3d& L) {
    ChBox box(L);

    // Volume and gyration (unit mass inertia) of a solid box with side lengths L
    double V = L.x() * L.y() * L.z();
    double Jxx = (L.y() * L.y() + L.z() * L.z()) / 12;
    double Jyy = (L.z() * L.z() + L.x() * L.x()) / 12;
    double Jzz = (L.x() * L.x() + L.y() * L.y()) / 12;

    EXPECT_NEAR(box.GetVolume(), V, ABS_ERR);
    EXPECT_NEAR(box.GetVolume(), ChBox::CalcVolume(L), ABS_ERR);

    auto J = box.GetGyration();
    EXPECT_NEAR(J(0, 0), Jxx, ABS_ERR);
    EXPECT_NEAR(J(1, 1), Jyy, ABS_ERR);
    EXPECT_NEAR(J(2, 2), Jzz, ABS_ERR);
    EXPECT_NEAR(J(0, 1), 0.0, ABS_ERR);
    EXPECT_NEAR(J(0, 2), 0.0, ABS_ERR);
    EXPECT_NEAR(J(1, 2), 0.0, ABS_ERR);

    auto Jd = box.GetGyrationXX();
    EXPECT_NEAR(Jd.x(), Jxx, ABS_ERR);
    EXPECT_NEAR(Jd.y(), Jyy, ABS_ERR);
    EXPECT_NEAR(Jd.z(), Jzz, ABS_ERR);

    auto aabb = box.GetBoundingBox();
    EXPECT_NEAR(aabb.min.x(), -L.x() / 2, ABS_ERR);
    EXPECT_NEAR(aabb.min.y(), -L.y() / 2, ABS_ERR);
    EXPECT_NEAR(aabb.min.z(), -L.z() / 2, ABS_ERR);
    EXPECT_NEAR(aabb.max.x(), +L.x() / 2, ABS_ERR);
    EXPECT_NEAR(aabb.max.y(), +L.y() / 2, ABS_ERR);
    EXPECT_NEAR(aabb.max.z(), +L.z() / 2, ABS_ERR);

    double R = std::sqrt(L.x() * L.x() + L.y() * L.y() + L.z() * L.z()) / 2;
    EXPECT_NEAR(box.GetBoundingSphereRadius(), R, ABS_ERR);
}

// Sides under 1 (half-lengths truncate to 0 if the vector is converted to integers)
TEST(ChBoxTest, small_box) {
    CheckBox(ChVector3d(0.3, 0.1, 0.1));
    CheckBox(ChVector3d(0.4, 0.11, 0.2));

    ChBox box(0.3, 0.1, 0.1);
    EXPECT_NEAR(box.GetVolume(), 0.003, ABS_ERR);
    EXPECT_NEAR(box.GetBoundingSphereRadius(), std::sqrt(0.11) / 2, ABS_ERR);
}

// Sides over 1 with non-integer half-lengths
TEST(ChBoxTest, odd_box) {
    CheckBox(ChVector3d(3, 5, 7));
    EXPECT_NEAR(ChBox(3, 5, 7).GetVolume(), 105.0, ABS_ERR);
}

// Sides over 1 with integer half-lengths
TEST(ChBoxTest, even_box) {
    CheckBox(ChVector3d(2, 4, 6));
    EXPECT_NEAR(ChBox(2, 4, 6).GetVolume(), 48.0, ABS_ERR);
}
