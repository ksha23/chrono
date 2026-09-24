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
// Friction slot test: a loose block of spheres with multi-step friction and
// rolling resistance is dropped onto a plane and allowed to scatter, so contact
// slots are claimed and released many times. At several points in time, every
// free contact slot (no partner) must carry a zero tangential history, and
// released slots must have been observed.
// =============================================================================

#include "gtest/gtest.h"
#include <algorithm>
#include <climits>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "chrono_dem/physics/ChSystemDem.h"

using namespace chrono;
using namespace chrono::dem;

namespace {

// Contact slot data read back from a contact history file
struct SlotData {
    std::vector<unsigned int> partners;
    std::vector<float> history;  // 3 floats per slot
};

bool ReadContactHistory(const std::string& filename, SlotData& data, unsigned int& slots_per_sphere) {
    std::ifstream in(filename);
    std::string line;
    if (!std::getline(in, line))
        return false;
    std::istringstream header(line);
    std::string key_p, key_h;
    unsigned int n_p = 0, n_h = 0;
    header >> key_p >> n_p >> key_h >> n_h;
    if (key_p != "partners" || key_h != "history" || n_p == 0 || n_p != n_h)
        return false;
    slots_per_sphere = n_p;

    data.partners.clear();
    data.history.clear();
    while (std::getline(in, line)) {
        if (line.empty())
            continue;
        std::istringstream row(line);
        for (unsigned int i = 0; i < n_p; i++) {
            unsigned long long p;
            row >> p;
            data.partners.push_back((unsigned int)p);
        }
        for (unsigned int i = 0; i < 3 * n_h; i++) {
            float h;
            row >> h;
            data.history.push_back(h);
        }
        if (row.fail())
            return false;
    }
    return true;
}

}  // namespace

TEST(demFrictionSlots, freeSlotsHaveZeroHistory) {
    const float radius = 0.5f;
    const float density = 2.5f;
    const ChVector3f box(30.f, 30.f, 30.f);

    ChSystemDem dem_sys(radius, density, box);
    dem_sys.SetGravitationalAcceleration(ChVector3f(0, 0, -980.f));
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    dem_sys.SetKn_SPH2SPH(1e7);
    dem_sys.SetKn_SPH2WALL(1e7);
    dem_sys.SetGn_SPH2SPH(1e3);
    dem_sys.SetGn_SPH2WALL(1e3);
    dem_sys.SetKt_SPH2SPH(2e6);
    dem_sys.SetKt_SPH2WALL(1e6);
    dem_sys.SetGt_SPH2SPH(50);
    dem_sys.SetGt_SPH2WALL(50);
    dem_sys.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(0.5f);
    dem_sys.SetRollingMode(CHDEM_ROLLING_MODE::SCHWARTZ);
    dem_sys.SetRollingCoeff_SPH2SPH(0.01f);
    dem_sys.SetRollingCoeff_SPH2WALL(0.01f);

    // Loose block of spheres with a deterministic velocity pattern, so that contacts form and break
    std::vector<ChVector3f> pos;
    std::vector<ChVector3f> vel;
    const float spacing = 2.2f * radius;
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            for (int k = 0; k < 6; k++) {
                pos.push_back(ChVector3f((i - 5.5f) * spacing, (j - 5.5f) * spacing, -8.f + k * spacing));
                vel.push_back(ChVector3f(20.f * ((i * 7 + k * 3) % 5 - 2), 20.f * ((j * 5 + k) % 5 - 2), -50.f));
            }
        }
    }
    dem_sys.SetParticles(pos, vel);

    // Sphere-to-wall contacts also use contact slots
    dem_sys.CreateBCPlane(ChVector3f(0, 0, -12.f), ChVector3f(0, 0, 1), false);

    const float step_size = 1e-4f;
    dem_sys.SetFixedStepSize(step_size);
    dem_sys.SetBDFixed(true);
    dem_sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
    dem_sys.Initialize();

    const std::string hist_file = "utest_DEM_frictionslots_history.txt";
    SlotData prev, curr;
    unsigned int slots = 0;
    size_t released = 0;
    size_t max_occupied = 0;

    const int n_checks = 8;
    for (int c = 0; c < n_checks; c++) {
        dem_sys.AdvanceSimulation(0.02f);
        dem_sys.WriteContactHistoryFile(hist_file);
        ASSERT_TRUE(ReadContactHistory(hist_file, curr, slots));
        ASSERT_EQ(curr.partners.size(), slots * pos.size());
        ASSERT_EQ(curr.history.size(), 3 * curr.partners.size());

        size_t occupied = 0;
        size_t bad = 0;
        for (size_t s = 0; s < curr.partners.size(); s++) {
            if (curr.partners[s] != UINT_MAX) {
                occupied++;
                continue;
            }
            if (curr.history[3 * s] != 0 || curr.history[3 * s + 1] != 0 || curr.history[3 * s + 2] != 0)
                bad++;
            if (c > 0 && prev.partners[s] != UINT_MAX)
                released++;
        }
        std::cout << "t = " << (c + 1) * 0.02f << "  occupied slots = " << occupied << "  released so far = " << released << std::endl;

        max_occupied = std::max(max_occupied, occupied);
        ASSERT_EQ(bad, 0u) << "free contact slots with nonzero history at check " << c;
        prev = curr;
    }

    // The scenario must actually claim and free slots, otherwise it does not exercise the reset path
    ASSERT_GT(max_occupied, 0u);
    ASSERT_GT(released, 0u);

    std::remove(hist_file.c_str());
}
