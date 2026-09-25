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
// 3. Pairs of spheres that barely touch, for each sphere-sphere force kernel. The float
//    contact distance of such a pair can round to a slightly negative penetration; all
//    states must stay finite.
// 4. Free fall with the extended Taylor integrator, which is exact for a constant
//    acceleration.
// =============================================================================

#include <algorithm>
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

    // Always select the model explicitly (see the note above the tests)
    sys.UseMaterialBasedModel(mat_based);
    if (mat_based) {
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

// Pairs of spheres whose centers are 2R apart in user units, in many directions, with the default (minimum) length
// unit as in the DEM demos (here R is about 2.7e7 SU). After conversion to integer SU, each pair penetrates or is
// separated by up to about a hundred SU, i.e. it barely touches. The float contact distance of such a pair has an
// error of a few SU (about 1e-7 * 2R) and can give a slightly negative penetration, which must not produce a NaN.
void RunNearTouchingPairs(bool mat_based, CHDEM_FRICTION_MODE friction) {
    const int n_side = 12;                    // pairs per side of the lattice
    const float spacing = 6 * radius;         // lattice spacing
    const float half = n_side * spacing / 2;  // half lattice size

    ChSystemDem sys(radius, density, ChVector3f(2 * half + 8, 2 * half + 8, 2 * half + 8));

    std::vector<ChVector3f> pos;
    const int npairs = n_side * n_side * n_side;
    int k = 0;
    for (int i = 0; i < n_side; i++) {
        for (int j = 0; j < n_side; j++) {
            for (int l = 0; l < n_side; l++, k++) {
                // direction on a golden-angle spiral
                double cz = 1 - 2 * (k + 0.5) / npairs;
                double sz = std::sqrt(1 - cz * cz);
                double phi = k * 2.399963229728653;
                ChVector3f dir((float)(sz * std::cos(phi)), (float)(sz * std::sin(phi)), (float)cz);
                ChVector3f a(-half + spacing * (i + 0.5f), -half + spacing * (j + 0.5f), -half + spacing * (l + 0.5f));
                pos.push_back(a);
                pos.push_back(a + 2 * radius * dir);
            }
        }
    }
    sys.SetParticles(pos);

    sys.UseMaterialBasedModel(mat_based);
    if (mat_based) {
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
    sys.SetFixedStepSize(1e-5f);
    sys.SetBDFixed(true);
    sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
    sys.Initialize();

    for (int step = 0; step < 5; step++)
        sys.AdvanceSimulation(1e-5f);

    size_t nonfinite = 0;
    double vmax = 0;
    for (size_t n = 0; n < pos.size(); n++) {
        ChVector3f p = sys.GetParticlePosition((int)n);
        ChVector3f v = sys.GetParticleVelocity((int)n);
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) || !std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z()))
            nonfinite++;
        else
            vmax = std::max(vmax, (double)v.Length());
    }
    unsigned int ncontacts = sys.GetNumContacts();
    std::cout << npairs << " pairs, " << ncontacts << " in contact, non-finite states " << nonfinite << ", max speed " << vmax << " (free fall " << grav * 5e-5 << ")" << std::endl;
    EXPECT_EQ(nonfinite, 0u);
    // about half of the pairs touch, the others are separated by a few SU (contacts are counted only with friction)
    if (friction != CHDEM_FRICTION_MODE::FRICTIONLESS)
        EXPECT_GT(ncontacts, npairs / 4u);
    // the contact forces are negligible: every sphere moves as in free fall
    EXPECT_NEAR(vmax, grav * 5e-5, 1e-3 * grav * 5e-5);
}

}  // namespace

// Note: every test selects the contact model explicitly with UseMaterialBasedModel. At aa3922df4a the constructor
// does not initialize the device-side use_mat_based flag, so a system created after a material-based one in the same
// process can inherit the flag from reused managed memory and then does not advance. With the explicit call the
// tests pass in any order (checked with --gtest_shuffle).
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

    sys.UseMaterialBasedModel(false);
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

TEST(demContactForces, near_touching_user_friction) {
    RunNearTouchingPairs(false, CHDEM_FRICTION_MODE::MULTI_STEP);
}

TEST(demContactForces, near_touching_matbased_friction) {
    RunNearTouchingPairs(true, CHDEM_FRICTION_MODE::MULTI_STEP);
}

TEST(demContactForces, near_touching_matbased_frictionless) {
    RunNearTouchingPairs(true, CHDEM_FRICTION_MODE::FRICTIONLESS);
}

// Extended Taylor: x += h (v + a h / 2), v += a h. For a constant acceleration this is exact, z(t) = z0 - g t^2 / 2,
// while forward Euler and centered difference are off by -+ g t h / 2 (0.0098 here).
TEST(demContactForces, free_fall_extended_taylor) {
    ChSystemDem sys(radius, density, ChVector3f(10, 10, 60));
    const float z0 = 25;
    std::vector<ChVector3f> pos = {ChVector3f(0.37f, 0.21f, z0), ChVector3f(-2.63f, 0.21f, z0)};
    sys.SetParticles(pos);
    sys.UseMaterialBasedModel(false);
    sys.SetKn_SPH2SPH(kn);
    sys.SetKn_SPH2WALL(kn);
    sys.SetGn_SPH2SPH(5e4);
    sys.SetGn_SPH2WALL(5e4);
    sys.SetGravitationalAcceleration(ChVector3f(0, 0, -grav));
    sys.SetFrictionMode(CHDEM_FRICTION_MODE::FRICTIONLESS);
    sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    const float h = 1e-4f;
    const int nsteps = 2000;
    sys.SetFixedStepSize(h);
    sys.SetBDFixed(true);
    sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);
    sys.Initialize();

    for (int n = 0; n < nsteps; n++)
        sys.AdvanceSimulation(h);

    const double t = (double)nsteps * h;
    const double z_exact = z0 - grav * t * t / 2;
    // (the system may reorder the spheres; match them by x)
    for (int i = 0; i < 2; i++) {
        ChVector3f p = sys.GetParticlePosition(i);
        ChVector3f v = sys.GetParticleVelocity(i);
        const ChVector3f& p0 = std::abs(p.x() - pos[0].x()) < 1 ? pos[0] : pos[1];
        std::cout << std::setprecision(8) << "sphere " << i << ": z " << p.z() << ", exact " << z_exact << ", vz " << v.z() << ", exact " << -grav * t << std::endl;
        EXPECT_NEAR(p.z(), z_exact, 1e-4);
        EXPECT_NEAR(v.z(), -grav * t, 1e-5 * grav * t);
        EXPECT_NEAR(p.x(), p0.x(), 1e-5);
        EXPECT_NEAR(p.y(), p0.y(), 1e-5);
    }
}
