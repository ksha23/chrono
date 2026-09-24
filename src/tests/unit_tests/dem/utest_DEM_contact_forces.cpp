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
// Validation test: static sphere-sphere contact forces in Chrono::DEM
//
// 1. A vertical column of spheres resting on a fixed sphere. At rest, the contact
//    between spheres i and i+1 carries the weight of the spheres above it, so its
//    penetration must match the closed-form Hertz law of the selected contact model.
//    Run for each sphere-sphere force kernel: user (Hertz) model with friction, and
//    material-based model with or without friction. (Frictionless mode always uses the
//    material-based kernel.)
// 2. A settled bed of spheres on a force-tracking plane. At rest, the plane carries
//    the weight of the bed, the kinetic energy is negligible, and the mean height
//    stays at its reference value.
// =============================================================================

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_dem/physics/ChSystemDem.h"

using namespace chrono;
using namespace chrono::dem;

namespace {

const float radius = 1.0f;
const float density = 1.0f;
const float grav = 980.0f;
const double kn = 1e8;        // user model normal stiffness
const double youngs = 1e9;    // material-based model Young's modulus
const double poisson = 0.3;   // material-based model Poisson ratio
const unsigned int nsph = 6;  // spheres in the column (the bottom one is fixed)
const float x0 = 0.37f;       // column position, away from subdomain boundaries
const float y0 = 0.21f;

// Penetration of a contact carrying the force F, for each contact model
double HertzPenetration(bool mat_based, double F) {
    if (mat_based) {
        // F = 4/3 E_eff sqrt(R_eff) delta^1.5, R_eff = R / 2, E_eff = E / (2 (1 - nu^2))
        double E_eff = youngs / (2 * (1 - poisson * poisson));
        return std::pow(F / (4.0 / 3.0 * E_eff * std::sqrt(radius / 2.0)), 2.0 / 3.0);
    }
    // F = kn R (delta / R)^1.5
    return radius * std::pow(F / (kn * radius), 2.0 / 3.0);
}

void RunColumn(bool mat_based, CHDEM_FRICTION_MODE friction) {
    ChSystemDem sys(radius, density, ChVector3f(20, 20, 40));

    // Fine length unit so that position quantization is well below the tolerance
    sys.SetPsiFactors(32, 128);

    std::vector<ChVector3f> pos;
    std::vector<bool> fixed;
    for (unsigned int i = 0; i < nsph; i++) {
        pos.push_back(ChVector3f(x0, y0, -15 + 2 * radius * i));
        fixed.push_back(i == 0);
    }
    sys.SetParticles(pos);
    sys.SetParticleFixed(fixed);

    if (mat_based) {
        sys.UseMaterialBasedModel(true);
        sys.SetYoungModulus_SPH(youngs);
        sys.SetYoungModulus_WALL(youngs);
        sys.SetPoissonRatio_SPH(poisson);
        sys.SetPoissonRatio_WALL(poisson);
        sys.SetRestitution_SPH(0.1);
        sys.SetRestitution_WALL(0.1);
    } else {
        sys.SetKn_SPH2SPH(kn);
        sys.SetKn_SPH2WALL(kn);
        sys.SetGn_SPH2SPH(5e4);
        sys.SetGn_SPH2WALL(5e4);
        sys.SetKt_SPH2SPH(kn);
        sys.SetKt_SPH2WALL(kn);
        sys.SetGt_SPH2SPH(5e4);
        sys.SetGt_SPH2WALL(5e4);
    }
    sys.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    sys.SetStaticFrictionCoeff_SPH2WALL(0.5f);

    sys.SetGravitationalAcceleration(ChVector3f(0, 0, -grav));
    sys.SetFrictionMode(friction);
    sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    sys.SetFixedStepSize(2e-5f);
    sys.SetBDFixed(true);
    sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
    sys.Initialize();

    sys.AdvanceSimulation(0.3f);

    const double weight = 4.0 / 3.0 * CH_PI * radius * radius * radius * density * grav;
    for (unsigned int i = 0; i + 1 < nsph; i++) {
        ChVector3f p0 = sys.GetParticlePosition(i);
        ChVector3f p1 = sys.GetParticlePosition(i + 1);
        double delta = 2 * radius - (p1 - p0).Length();
        double expected = HertzPenetration(mat_based, (nsph - 1 - i) * weight);
        std::cout << "contact " << i << ": penetration " << delta << ", Hertz " << expected << ", rel. error " << (delta - expected) / expected << std::endl;
        EXPECT_NEAR(delta, expected, 0.02 * expected);
        // the column stays straight
        EXPECT_NEAR(p1.x(), x0, 1e-3);
        EXPECT_NEAR(p1.y(), y0, 1e-3);
        // at rest
        EXPECT_LT(sys.GetParticleVelocity(i + 1).Length(), 1e-2);
    }
}

}  // namespace

// Note: the bed test runs first. A ChSystemDem created after a material-based one in the same process does
// not advance (observed at aa3922df4a), so the material-based cases come last.
TEST(demContactForces, settled_bed) {
    const float box = 24;
    ChSystemDem sys(radius, density, ChVector3f(box, box, 40));

    // Loose lattice of spheres that falls onto a force-tracking floor plane
    std::vector<ChVector3f> pos;
    utils::ChHCPSampler<float> sampler(2.1f * radius);
    ChVector3f center(0, 0, -14);
    while (center.z() < -2) {
        auto layer = sampler.SampleBox(center, ChVector3f(box / 2 - 3 * radius, box / 2 - 3 * radius, 0));
        pos.insert(pos.end(), layer.begin(), layer.end());
        center.z() += 2.1f * radius;
    }
    sys.SetParticles(pos);

    size_t floor = sys.CreateBCPlane(ChVector3f(0, 0, -16), ChVector3f(0, 0, 1), true);

    sys.SetKn_SPH2SPH(kn);
    sys.SetKn_SPH2WALL(kn);
    sys.SetGn_SPH2SPH(2e4);
    sys.SetGn_SPH2WALL(2e4);
    sys.SetKt_SPH2SPH(kn);
    sys.SetKt_SPH2WALL(kn);
    sys.SetGt_SPH2SPH(2e4);
    sys.SetGt_SPH2WALL(2e4);
    sys.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    sys.SetStaticFrictionCoeff_SPH2WALL(0.5f);

    sys.SetGravitationalAcceleration(ChVector3f(0, 0, -grav));
    sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    sys.SetFixedStepSize(5e-5f);
    sys.SetBDFixed(true);
    sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
    sys.Initialize();

    sys.AdvanceSimulation(0.6f);

    const size_t n = sys.GetNumParticles();
    double zmean = 0;
    size_t nonfinite = 0;
    for (size_t i = 0; i < n; i++) {
        ChVector3f p = sys.GetParticlePosition((int)i);
        ChVector3f v = sys.GetParticleVelocity((int)i);
        if (!std::isfinite(p.z()) || !std::isfinite(v.Length()))
            nonfinite++;
        zmean += p.z();
    }
    zmean /= n;
    ASSERT_EQ(nonfinite, 0u);

    ChVector3f reaction;
    ASSERT_TRUE(sys.GetBCReactionForces(floor, reaction));
    const double weight = n * 4.0 / 3.0 * CH_PI * radius * radius * radius * density * grav;
    double ke = sys.GetParticlesKineticEnergy();

    std::cout << std::setprecision(8) << "spheres " << n << ", zmean " << zmean << ", floor force " << reaction.z() << " (weight " << weight << "), KE " << ke << std::endl;

    // static equilibrium: the floor carries the bed
    EXPECT_NEAR(std::abs(reaction.z()), weight, 0.01 * weight);
    EXPECT_NEAR(reaction.x(), 0, 0.01 * weight);
    EXPECT_NEAR(reaction.y(), 0, 0.01 * weight);
    // at rest
    EXPECT_LT(ke, 1e-4 * weight * radius);
    // reference mean height of the settled bed (aa3922df4a, CUDA); the bed is about 6 layers deep
    const double zmean_ref = -10.0109;
    EXPECT_NEAR(zmean, zmean_ref, 5e-3 * radius);
}

TEST(demContactForces, column_user_friction) {
    RunColumn(false, CHDEM_FRICTION_MODE::MULTI_STEP);
}

TEST(demContactForces, column_matbased_friction) {
    RunColumn(true, CHDEM_FRICTION_MODE::MULTI_STEP);
}

TEST(demContactForces, column_matbased_frictionless) {
    RunColumn(true, CHDEM_FRICTION_MODE::FRICTIONLESS);
}
