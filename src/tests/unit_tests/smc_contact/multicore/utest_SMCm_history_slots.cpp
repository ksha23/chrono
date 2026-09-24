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
// Check the tangential contact history bookkeeping of the Chrono::Multicore SMC
// solver (MultiStep tangential displacement model). A large sphere, added last
// so that it owns the history of all its contacts, is placed in contact with
// many smaller spheres at once. After one step, every one of these contacts must
// own exactly one history slot on the large sphere, regardless of the number of
// OpenMP threads used to process the contacts.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#define SMC_MULTICORE
#include "../utest_SMC.h"

#include "chrono_multicore/ChDataManager.h"

#ifdef _OPENMP
    #include <omp.h>
#endif

// Count, for each small sphere, the number of history slots of the large sphere that refer to it.
static std::vector<int> CountSlots(ChSystemMulticoreSMC& sys, int num_small) {
    int center = num_small;  // index of the large sphere (added last)
    const auto& neigh = sys.data_manager->host_data.shear_neigh;
    std::vector<int> count(num_small, 0);
    for (int i = 0; i < max_shear; i++) {
        int other = neigh[max_shear * center + i].x;
        if (other >= 0 && other < num_small)
            count[other]++;
    }
    return count;
}

TEST(ChronoMulticore, SMC_history_slots) {
    const int num_small = 18;  // must not exceed max_shear
    const double R = 3.0;      // large sphere radius
    const double r = 0.5;      // small sphere radius
    const double overlap = 1e-3;
    const int num_trials = 20;

    int num_threads = 16;
#ifdef _OPENMP
    num_threads = std::max(2, std::min(num_threads, omp_get_num_procs()));
#endif

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetYoungModulus(1e7f);
    mat->SetFriction(0.5f);

    int failed_trials = 0;
    for (int trial = 0; trial < num_trials; trial++) {
        ChSystemMulticoreSMC sys;
        SetSimParameters(&sys, ChVector3d(0, 0, 0), ChSystemSMC::ContactForceModel::Hertz, ChSystemSMC::TangentialDisplacementModel::MultiStep);
        sys.SetNumThreads(num_threads);

        // Small spheres on a Fibonacci lattice around the origin, each slightly overlapping the large sphere.
        for (int i = 0; i < num_small; i++) {
            double z = 1 - (2 * i + 1.0) / num_small;
            double rho = std::sqrt(1 - z * z);
            double phi = i * CH_PI * (3 - std::sqrt(5.0));
            ChVector3d dir(rho * std::cos(phi), rho * std::sin(phi), z);
            AddSphere(&sys, mat, r, 1.0, (R + r - overlap) * dir, ChVector3d(0, 0, 0));
        }
        AddSphere(&sys, mat, R, 100.0, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));

        sys.DoStepDynamics(1e-5);
        ASSERT_EQ(sys.GetNumContacts(), (unsigned int)num_small);

        auto count = CountSlots(sys, num_small);
        bool ok = std::all_of(count.begin(), count.end(), [](int c) { return c == 1; });
        if (!ok)
            failed_trials++;
    }

    EXPECT_EQ(failed_trials, 0) << "contact history slots lost or duplicated in " << failed_trials << " of " << num_trials << " trials (" << num_threads << " threads)";
}
