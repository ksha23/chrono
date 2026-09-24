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
// Settled-bed and checkpoint tests for the Chrono::Dem solver state that only
// kernels touch during a step (force accumulators, contact partners and
// friction history). The host reads this state through explicit copies in the
// accessors and output functions; these tests check those paths against each
// other and against the physics of a settled bed.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_dem/physics/ChSystemDem.h"

#include "ut_dem_utils.h"

using namespace chrono;
using namespace chrono::dem;

namespace {

const float radius = 1.f;
const float density = 2.5f;
const float g = 980.f;
const float box = 30.f;

float SphereMass() {
    return 4.f / 3.f * (float)CH_PI * radius * radius * radius * density;
}

void SetupSystem(ChSystemDem& sys, CHDEM_TIME_INTEGRATOR integrator) {
    sys.SetGravitationalAcceleration(ChVector3f(0, 0, -g));
    sys.SetKn_SPH2SPH(5e7f);
    sys.SetKn_SPH2WALL(5e7f);
    sys.SetGn_SPH2SPH(2e4f);
    sys.SetGn_SPH2WALL(2e4f);
    sys.SetKt_SPH2SPH(2e7f);
    sys.SetKt_SPH2WALL(2e7f);
    sys.SetGt_SPH2SPH(50.f);
    sys.SetGt_SPH2WALL(50.f);
    sys.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    sys.SetStaticFrictionCoeff_SPH2WALL(0.5f);
    sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    sys.SetTimeIntegrator(integrator);
    sys.SetBDFixed(true);
    sys.SetFixedStepSize(5e-5f);
    sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
}

// Count the contact partner entries in a contact history file ("partners 12 history 12" header, then one line per
// sphere with 12 partner IDs followed by 12 history vectors), and among them the entries whose partner is a sphere
// (ID below n). Also return the largest history magnitude.
void ReadHistoryFile(const std::string& filename, unsigned int n, unsigned int& num_entries, unsigned int& num_sphere_entries, double& max_history) {
    std::ifstream f(filename);
    std::string line;
    std::getline(f, line);
    num_entries = 0;
    num_sphere_entries = 0;
    max_history = 0;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        unsigned int id;
        for (int i = 0; i < 12 && (ls >> id); i++) {
            if (id != (unsigned int)-1)
                num_entries++;
            if (id < n)
                num_sphere_entries++;
        }
        float x, y, z;
        while (ls >> x >> y >> z)
            max_history = std::max(max_history, std::sqrt((double)x * x + (double)y * y + (double)z * z));
    }
}

std::string ReadFile(const std::string& filename) {
    std::ifstream f(filename, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

const float plane_z = -box / 2 + 2 * radius;

// Add a plane just above the bottom wall (tracking its reaction force) and an HCP lattice a little above it, then let
// the lattice settle under gravity. Return the number of particles.
unsigned int SettleBed(ChSystemDem& sys, size_t& plane) {
    plane = sys.CreateBCPlane(ChVector3f(0, 0, plane_z), ChVector3f(0, 0, 1), true);

    chrono::utils::ChHCPSampler<float> sampler(2.1f * radius);
    ChVector3f hdims(box / 2 - 2 * radius, box / 2 - 2 * radius, box / 8);
    ChVector3f center(0.f, 0.f, plane_z + 1.5f * radius + hdims.z());
    std::vector<ChVector3f> points = sampler.SampleBox(center, hdims);
    sys.SetParticles(points);

    sys.Initialize();
    for (int frame = 0; frame < 10; frame++)
        sys.AdvanceSimulation(0.05f);
    return (unsigned int)points.size();
}

// Rows of a contact history file after the header line, sorted (to compare two files up to a particle permutation).
std::vector<std::string> SortedHistoryRows(const std::string& filename) {
    std::ifstream f(filename);
    std::string line;
    std::getline(f, line);
    std::vector<std::string> rows;
    while (std::getline(f, line)) {
        if (line.find_first_not_of(' ') != std::string::npos)
            rows.push_back(line);
    }
    std::sort(rows.begin(), rows.end());
    return rows;
}

void RunSettledBed(CHDEM_TIME_INTEGRATOR integrator, const std::string& tag) {
    ChSystemDem sys(radius, density, ChVector3f(box, box, box));
    SetupSystem(sys, integrator);
    size_t plane;
    unsigned int n = SettleBed(sys, plane);

    // 1. The plane carries the weight of the bed (the reaction force is reported with the sign of gravity). Friction
    // on the side walls carries about 1% of it.
    ChVector3f reaction;
    ASSERT_TRUE(sys.GetBCReactionForces(plane, reaction));
    double weight = n * SphereMass() * g;
    EXPECT_NEAR(-reaction.z(), weight, 0.03 * weight) << tag;

    // 2. The bed has settled: the mean particle acceleration (net force per unit mass, read back from the
    // accumulators) is close to zero, and the particles move slowly.
    ChVector3d acc_mean(0);
    for (unsigned int i = 0; i < n; i++)
        acc_mean += ChVector3d(sys.GetParticleLinAcc(i));
    acc_mean /= n;
    EXPECT_LT(acc_mean.Length(), 0.02 * g) << tag;

    double ke = sys.GetParticlesKineticEnergy();
    EXPECT_LT(ke / n, 0.5 * SphereMass() * 10.0 * 10.0) << tag;  // rms speed below 10 cm/s

    // 3. The forces written by WriteParticleFile match the accelerations read one particle at a time.
    std::string csv = "DEM_device_state_" + tag + ".csv";
    sys.SetParticleOutputMode(CHDEM_OUTPUT_MODE::CSV);
    sys.SetParticleOutputFlags(CHDEM_OUTPUT_FLAGS::FORCE_COMPONENTS);
    sys.WriteParticleFile(csv);
    std::vector<float> fz = loadColumnCheckpoint(csv, 5);
    ASSERT_EQ(fz.size(), n) << tag;
    double m = SphereMass();
    double max_err = 0;
    for (unsigned int i = 0; i < n; i++) {
        double fz_acc = m * (sys.GetParticleLinAcc(i).z() + g);
        max_err = std::max(max_err, std::abs(fz_acc - fz[i]));
    }
    EXPECT_LT(max_err, 1e-3 * m * g) << tag;

    // 4. Contacts: every particle in the bed touches several others. The history file lists each contact partner
    // entry (sphere-sphere pairs twice, sphere-wall once), and GetNumContacts returns half their number.
    unsigned int nc = sys.GetNumContacts();
    EXPECT_GT(nc, n) << tag;
    EXPECT_LT(nc, 6 * n) << tag;

    std::string hst = "DEM_device_state_" + tag + ".hst";
    sys.WriteContactHistoryFile(hst);
    unsigned int num_entries, num_sphere_entries;
    double max_history;
    ReadHistoryFile(hst, n, num_entries, num_sphere_entries, max_history);
    EXPECT_EQ(num_entries / 2, nc) << tag;
    EXPECT_GT(max_history, 0) << tag;  // tangential history is carried between steps
    EXPECT_LT(max_history, radius) << tag;

    // The sphere-sphere entries, counted independently from the particle positions: every overlapping pair appears
    // once for each of its two spheres. The contact list is built at the start of the last step and the positions
    // are read at its end, so a pair that is just touching may be counted on one side only.
    std::vector<ChVector3f> pos(n);
    for (unsigned int i = 0; i < n; i++)
        pos[i] = sys.GetParticlePosition(i);
    unsigned int num_overlaps = 0;
    for (unsigned int i = 0; i < n; i++) {
        for (unsigned int j = i + 1; j < n; j++) {
            if ((pos[i] - pos[j]).Length() < 2 * radius)
                num_overlaps++;
        }
    }
    EXPECT_GT(num_overlaps, n) << tag;
    EXPECT_EQ(num_sphere_entries % 2, 0u) << tag;
    EXPECT_LE(std::abs((double)num_sphere_entries / 2 - num_overlaps), 0.01 * num_overlaps) << tag;

    // 5. Checkpoint round trip: a system restored from a checkpoint writes the same checkpoint.
    std::string cp1 = "DEM_device_state_" + tag + "_1.dat";
    std::string cp2 = "DEM_device_state_" + tag + "_2.dat";
    sys.WriteCheckpointFile(cp1);
    {
        ChSystemDem restored(cp1);
        restored.SetVerbosity(CHDEM_VERBOSITY::QUIET);
        restored.Initialize();
        EXPECT_EQ(restored.GetNumParticles(), n) << tag;
        EXPECT_EQ(restored.GetNumContacts(), nc) << tag;
        restored.WriteCheckpointFile(cp2);
    }
    // The parameters and the contact partner/history section must be identical. Particle positions are stored as
    // integers in simulation units, so the printed positions may differ in the last printed digit.
    std::string s1 = ReadFile(cp1);
    std::string s2 = ReadFile(cp2);
    size_t p1 = s1.find("CsvParticles"), h1 = s1.find("HstHistory");
    size_t p2 = s2.find("CsvParticles"), h2 = s2.find("HstHistory");
    ASSERT_NE(h1, std::string::npos) << tag;
    ASSERT_NE(h2, std::string::npos) << tag;
    EXPECT_TRUE(s1.substr(0, p1) == s2.substr(0, p2)) << tag << ": parameters changed in a checkpoint round trip";
    EXPECT_TRUE(s1.substr(h1) == s2.substr(h2)) << tag << ": contact history changed in a checkpoint round trip";
    std::istringstream ps1(s1.substr(p1, h1 - p1)), ps2(s2.substr(p2, h2 - p2));
    std::string line1, line2;
    std::getline(ps1, line1);  // "CsvParticles"
    std::getline(ps2, line2);
    std::getline(ps1, line1);  // column names
    std::getline(ps2, line2);
    EXPECT_EQ(line1, line2) << tag;
    double max_dev = 0;
    unsigned int rows = 0;
    while (std::getline(ps1, line1) && std::getline(ps2, line2)) {
        if (line1.empty() && line2.empty())
            continue;
        std::istringstream l1(line1), l2(line2);
        std::string c1, c2;
        while (std::getline(l1, c1, ',') && std::getline(l2, c2, ',')) {
            double v1 = std::stod(c1), v2 = std::stod(c2);
            max_dev = std::max(max_dev, std::abs(v1 - v2) / std::max(1.0, std::abs(v1)));
        }
        rows++;
    }
    EXPECT_EQ(rows, n) << tag;
    EXPECT_LT(max_dev, 1e-5) << tag;

    // 6. Restart with defragmentation (particles reordered by subdomain): the partner and history maps are permuted
    // row by row, so the file has the same rows, possibly in another order.
    std::string hst_defrag = "DEM_device_state_" + tag + "_defrag.hst";
    {
        ChSystemDem restored(cp1);
        restored.SetVerbosity(CHDEM_VERBOSITY::QUIET);
        restored.SetDefragmentOnInitialize(true);
        restored.Initialize();
        EXPECT_EQ(restored.GetNumContacts(), nc) << tag;
        restored.WriteContactHistoryFile(hst_defrag);
    }
    std::vector<std::string> rows_ref = SortedHistoryRows(hst);
    std::vector<std::string> rows_defrag = SortedHistoryRows(hst_defrag);
    EXPECT_EQ(rows_ref.size(), n) << tag;
    EXPECT_TRUE(rows_ref == rows_defrag) << tag << ": contact history rows changed by defragmentation";

    if (::testing::Test::HasFailure())
        return;  // keep the files for inspection
    std::remove(csv.c_str());
    std::remove(hst.c_str());
    std::remove(hst_defrag.c_str());
    std::remove(cp1.c_str());
    std::remove(cp2.c_str());
}

// Pairwise contact queries and the contact info file, on a settled bed with rolling resistance and contact recording.
void RunContactQueries() {
    ChSystemDem sys(radius, density, ChVector3f(box, box, box));
    SetupSystem(sys, CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    sys.SetRollingMode(CHDEM_ROLLING_MODE::SCHWARTZ);
    sys.SetRollingCoeff_SPH2SPH(0.05f);
    sys.SetRollingCoeff_SPH2WALL(0.05f);
    sys.SetRecordingContactInfo(true);
    size_t plane;
    unsigned int n = SettleBed(sys, plane);

    double m = SphereMass();
    std::vector<ChVector3f> pos(n);
    for (unsigned int i = 0; i < n; i++)
        pos[i] = sys.GetParticlePosition(i);

    unsigned int num_pairs = 0;     // sphere-sphere pairs (i < j) listed by getNeighbors
    unsigned int num_balanced = 0;  // interior spheres used in the force balance
    double max_newton = 0;          // largest |N_ij + N_ji| relative to |N_ij|
    double min_align = 1;           // smallest cosine between N_ij and p_i - p_j
    double max_balance = 0;         // largest |sum of contact forces - m (a + g)|
    double max_torque = 0;
    for (unsigned int i = 0; i < n; i++) {
        std::vector<unsigned int> neighbors;
        sys.getNeighbors(i, neighbors);
        bool interior = std::abs(pos[i].x()) < box / 2 - 2 * radius && std::abs(pos[i].y()) < box / 2 - 2 * radius && pos[i].z() > plane_z + 2 * radius;
        ChVector3d force_sum(0);
        for (unsigned int j : neighbors) {
            if (j >= n) {
                interior = false;  // touches a wall or the plane
                continue;
            }
            if (i < j)
                num_pairs++;
            ChVector3f N_ij = sys.getNormalForce(i, j);
            ChVector3f N_ji = sys.getNormalForce(j, i);
            ChVector3f F_ij = sys.getSlidingFrictionForce(i, j);
            max_torque = std::max(max_torque, (double)sys.getRollingFrictionTorque(i, j).Length());
            EXPECT_TRUE(std::isfinite(sys.getRollingVrot(i, j).Length()));
            EXPECT_TRUE(std::isfinite(sys.getRollingCharContactTime(i, j)));
            if (N_ij.Length() > 0) {
                max_newton = std::max(max_newton, (double)(N_ij + N_ji).Length() / N_ij.Length());
                ChVector3f d = pos[i] - pos[j];
                min_align = std::min(min_align, (double)N_ij.Dot(d) / (N_ij.Length() * d.Length()));
            }
            force_sum += ChVector3d(N_ij + F_ij);
        }
        if (interior) {
            ChVector3d acc(sys.GetParticleLinAcc(i));
            ChVector3d contact_force = m * (acc + ChVector3d(0, 0, g));
            max_balance = std::max(max_balance, (force_sum - contact_force).Length());
            num_balanced++;
        }
    }

    // Newton's third law, and normal forces along the line of centers.
    EXPECT_GT(num_pairs, n);
    EXPECT_LT(max_newton, 1e-4);
    EXPECT_GT(min_align, 0.99);
    // The net force on an interior sphere (from the accumulators) is the sum of its pairwise contact forces.
    EXPECT_GT(num_balanced, n / 4);
    EXPECT_LT(max_balance, 1e-3 * m * g);
    EXPECT_GT(max_torque, 0);

    // A sphere index outside the contact map is reported, not read out of bounds.
    std::vector<unsigned int> neighbors;
    EXPECT_THROW(sys.getNeighbors(n + 1, neighbors), std::out_of_range);

    // The contact info file has one row per sphere-sphere pair (i < j) with the values of the pairwise queries.
    std::string info = "DEM_device_state_contact_info.csv";
    sys.WriteContactInfoFile(info);
    std::ifstream f(info);
    std::string line;
    std::getline(f, line);
    EXPECT_EQ(line, "bi, bj, n_mag, fx, fy, fz, mx, my, mz");
    unsigned int rows = 0;
    double max_err = 0;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string c;
        std::vector<double> v;
        while (std::getline(ls, c, ','))
            v.push_back(std::stod(c));
        ASSERT_EQ(v.size(), 9u);
        unsigned int i = (unsigned int)v[0], j = (unsigned int)v[1];
        ASSERT_LT(i, j);
        ASSERT_LT(j, n);
        ChVector3f F = sys.getSlidingFrictionForce(i, j);
        ChVector3f M = sys.getRollingFrictionTorque(i, j);
        double err = std::abs(v[2] - sys.getNormalForce(i, j).Length());
        err = std::max(err, (ChVector3d(v[3], v[4], v[5]) - ChVector3d(F)).Length());
        err = std::max(err, (ChVector3d(v[6], v[7], v[8]) - ChVector3d(M)).Length() / radius);
        max_err = std::max(max_err, err);
        rows++;
    }
    EXPECT_EQ(rows, num_pairs);
    EXPECT_LT(max_err, 1e-4 * m * g);  // values are printed with 6 significant digits

    if (!::testing::Test::HasFailure())
        std::remove(info.c_str());
}

}  // namespace

TEST(demDeviceState, settledBedCenteredDifference) {
    RunSettledBed(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE, "cd");
}

TEST(demDeviceState, settledBedChung) {
    RunSettledBed(CHDEM_TIME_INTEGRATOR::CHUNG, "chung");
}

TEST(demDeviceState, contactPairQueries) {
    RunContactQueries();
}
