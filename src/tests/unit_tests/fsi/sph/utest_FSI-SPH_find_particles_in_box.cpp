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
// Unit test for ChFsiFluidSystemSPH::FindParticlesInBox.
//
// A small fluid box (with container BCE markers) is created and initialized. FindParticlesInBox is
// queried with an empty box, a box containing part of the fluid, a box containing all of it, and a
// rotated (oriented) box. Each result is checked against a brute-force host-side containment test on
// the SPH particle positions.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

class FindParticlesInBoxTest : public ::testing::Test {
  protected:
    void SetUp() override {
        m_fsi = std::make_unique<ChFsiProblemCartesian>(m_spacing, &m_sysMBS);
        m_fsi->SetVerbose(false);
        m_fsi->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

        ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fluid_props.density = 1000;
        fluid_props.viscosity = 1e-3;
        m_fsi->SetCfdSPH(fluid_props);

        ChFsiFluidSystemSPH::SPHParameters sph_params;
        sph_params.initial_spacing = m_spacing;
        m_fsi->SetSPHParameters(sph_params);
        m_fsi->SetStepSizeCFD(1e-4);
        m_fsi->SetStepsizeMBD(1e-4);

        // Fluid box 0.4 x 0.2 x 0.2 with a container (bottom and side walls)
        m_fsi->Construct(ChVector3d(0.4, 0.2, 0.2), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);
        m_fsi->Initialize();

        m_sysSPH = m_fsi->GetFluidSystemSPH();
        m_num_sph = m_sysSPH->GetNumFluidMarkers();
        auto pos = m_sysSPH->GetParticlePositions();  // SPH particles first
        m_pos.assign(pos.begin(), pos.begin() + m_num_sph);
    }

    // Brute-force containment test in the box frame, with an optional margin on the half-sizes.
    bool Inside(const ChVector3d& p, const ChFrame<>& frame, const ChVector3d& size, double margin) const {
        ChVector3d w = frame.TransformPointParentToLocal(p);
        ChVector3d hs = 0.5 * size + ChVector3d(margin);
        return std::abs(w.x()) <= hs.x() && std::abs(w.y()) <= hs.y() && std::abs(w.z()) <= hs.z();
    }

    // Query the fluid system and check the result against the brute-force count. Particles within a
    // tiny distance of the box faces are allowed either way (single-precision device positions).
    size_t Check(const ChFrame<>& frame, const ChVector3d& size) {
        const double tol = 1e-5;
        std::vector<int> found = m_sysSPH->FindParticlesInBox(frame, size);

        std::set<int> found_set(found.begin(), found.end());
        EXPECT_EQ(found_set.size(), found.size()) << "duplicate indices returned";

        size_t num_strict = 0;
        for (int i = 0; i < (int)m_num_sph; i++) {
            bool strict = Inside(m_pos[i], frame, size, -tol);
            bool loose = Inside(m_pos[i], frame, size, +tol);
            bool in_found = found_set.count(i) > 0;
            if (strict) {
                num_strict++;
                EXPECT_TRUE(in_found) << "particle " << i << " inside the box but not returned";
            }
            if (!loose) {
                EXPECT_FALSE(in_found) << "particle " << i << " outside the box but returned";
            }
        }
        for (int i : found) {
            EXPECT_GE(i, 0);
            EXPECT_LT(i, (int)m_num_sph);
        }

        std::cout << "  box at " << frame.GetPos() << " size " << size << ": returned " << found.size() << ", brute force " << num_strict << " (of " << m_num_sph
                  << " SPH particles)" << std::endl;
        return found.size();
    }

    double m_spacing = 0.02;
    ChSystemSMC m_sysMBS;
    std::unique_ptr<ChFsiProblemCartesian> m_fsi;
    std::shared_ptr<ChFsiFluidSystemSPH> m_sysSPH;
    size_t m_num_sph = 0;
    std::vector<ChVector3d> m_pos;
};

TEST_F(FindParticlesInBoxTest, empty_box) {
    ASSERT_GT(m_num_sph, 0u);
    auto n = Check(ChFrame<>(ChVector3d(5, 5, 5), QUNIT), ChVector3d(0.1, 0.1, 0.1));
    EXPECT_EQ(n, 0u);
}

TEST_F(FindParticlesInBoxTest, partial_box) {
    ASSERT_GT(m_num_sph, 0u);
    auto n = Check(ChFrame<>(ChVector3d(0.053, 0.007, 0.101), QUNIT), ChVector3d(0.131, 0.093, 0.077));
    EXPECT_GT(n, 0u);
    EXPECT_LT(n, m_num_sph);
}

TEST_F(FindParticlesInBoxTest, all_particles) {
    ASSERT_GT(m_num_sph, 0u);
    auto n = Check(ChFrame<>(ChVector3d(0, 0, 0.1), QUNIT), ChVector3d(1, 1, 1));
    EXPECT_EQ(n, m_num_sph);
}

TEST_F(FindParticlesInBoxTest, rotated_box) {
    ASSERT_GT(m_num_sph, 0u);
    ChQuaterniond rot = QuatFromAngleZ(CH_PI / 6) * QuatFromAngleX(CH_PI / 9);
    auto n = Check(ChFrame<>(ChVector3d(-0.041, 0.013, 0.087), rot), ChVector3d(0.211, 0.057, 0.123));
    EXPECT_GT(n, 0u);
    EXPECT_LT(n, m_num_sph);
}
