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
// Unit test for the extended (halo) part of SPH active domains with several FSI
// bodies.
//
// Each active domain has an extended AABB, inflated by an envelope. Particles in
// the envelope are not integrated, but they are kept in the neighbor search, so
// that active particles near a domain face still see a full kernel support. If
// the halo were dropped, active particles near that face would lose neighbors.
//
// Setup: a CRM bed and three fixed bodies in a row along x. P (center) and Q
// (at +x) have identical active domains, and the bed looks the same around both
// (walls and other domains are out of kernel reach). R (at -x) has a tall and
// wide domain, so that the bounding box of the three active AABBs contains the
// whole halo of P, but not the +x part of the halo of Q. An implementation that
// dropped the extended AABBs from any shortcut test, for example one based on
// the active AABBs only, would lose that part of the halo of Q while keeping the
// halo of P.
//
// Observable: after a few steps, each particle in the domain of P must have the
// same velocity (up to round-off) as the particle at the same relative position
// in the domain of Q.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

int num_failures = 0;

void Check(const std::string& name, bool cond) {
    if (cond) {
        std::cout << "  ok:   " << name << std::endl;
    } else {
        std::cout << "  FAIL: " << name << std::endl;
        num_failures++;
    }
}

bool Inside(const ChVector3d& p, const ChAABB& aabb) {
    return p.x() >= aabb.min.x() && p.x() <= aabb.max.x() &&  //
           p.y() >= aabb.min.y() && p.y() <= aabb.max.y() &&  //
           p.z() >= aabb.min.z() && p.z() <= aabb.max.z();
}

bool Contains(const ChAABB& outer, const ChAABB& inner) {
    return Inside(inner.min, outer) && Inside(inner.max, outer);
}

ChAABB Shift(const ChAABB& aabb, const ChVector3d& d) {
    return ChAABB(aabb.min + d, aabb.max + d);
}

ChAABB Inflate(const ChAABB& aabb, double e) {
    return aabb.Inflate(ChVector3d(e, e, e));
}

int main(int argc, char* argv[]) {
    double spacing = 0.02;
    double L = 0.8;
    double H = 0.1;
    double body_radius = 0.02;
    double dt = 1e-4;
    int num_steps = 5;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.rheology_model = RheologyCRM::MU_OF_I;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    fsi.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Fixed bodies hovering above the bed, in a row along x. Domain faces are offset from the particle layers.
    // The offset between P and Q is a multiple of the particle spacing.
    double z = H + 0.03;
    ChVector3d pos_P(0.0, 0.0, z);
    ChVector3d pos_Q(0.28, 0.0, z);
    ChVector3d pos_R(-0.28, 0.0, z);
    ChAABB domain_PQ(ChVector3d(-0.051, -0.051, -0.101), ChVector3d(0.051, 0.051, 0.061));
    ChAABB domain_R(ChVector3d(-0.051, -0.161, -0.211), ChVector3d(0.051, 0.161, 0.171));

    for (const auto& [pos, domain] : {std::make_pair(pos_P, domain_PQ), std::make_pair(pos_Q, domain_PQ), std::make_pair(pos_R, domain_R)}) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(pos);
        body->SetMass(1.0);
        body->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
        body->SetFixed(true);
        sysMBS.AddBody(body);
        fsi.AddRigidBodySphere(body, ChVector3d(0, 0, 0), body_radius, false);
        fsi.SetActiveDomainBody(body, domain);
    }

    // Bed with an open top
    fsi.Construct(ChVector3d(L, L, H), ChVector3d(0, 0, 0), BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS | BoxSide::Y_NEG | BoxSide::Y_POS);

    fsi.Initialize();

    // Host replica of the absolute-frame domains. The kernel radius is 2 * h for the cubic spline kernel, and the envelope
    // of the extended AABBs is twice the kernel radius.
    double kernel_radius = 2 * sysSPH->GetKernelLength();
    double envelope = 2 * kernel_radius;
    ChAABB abs_P = Shift(domain_PQ, pos_P);
    ChAABB abs_Q = Shift(domain_PQ, pos_Q);
    ChAABB abs_R = Shift(domain_R, pos_R);
    ChAABB active_bounds = abs_P;
    active_bounds += abs_Q;
    active_bounds += abs_R;

    std::cout << "Kernel radius: " << kernel_radius << "  envelope: " << envelope << std::endl;

    // Guards against a layout that does not test what it is meant to test
    // (the bed floor is within kernel reach of both P and Q, at the same relative position).
    ChAABB reach_P = Inflate(abs_P, kernel_radius);
    ChAABB reach_Q = Inflate(abs_Q, kernel_radius);
    ChAABB reach_R = Inflate(abs_R, kernel_radius);
    double wall = L / 2 - spacing;
    Check("kernel reach of P and Q stays clear of the side walls",
          reach_Q.max.x() < wall && std::max(reach_P.max.y(), reach_Q.max.y()) < wall && std::min(reach_P.min.y(), reach_Q.min.y()) > -wall);
    Check("kernel reach of P and Q does not overlap R", reach_P.min.x() > reach_R.max.x() && reach_Q.min.x() > reach_R.max.x());
    Check("kernel reach of P and Q does not overlap", reach_P.max.x() < reach_Q.min.x());
    Check("halo of P is inside the bounding box of the active domains", Contains(active_bounds, Inflate(abs_P, envelope)));
    Check("halo of Q reaches past the bounding box of the active domains", abs_Q.max.x() + kernel_radius > active_bounds.max.x());

    size_t num_sph = fsi.GetNumSPHParticles();
    auto pos0 = sysSPH->GetParticlePositions();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    auto vel1 = sysSPH->GetParticleVelocities();

    // Pair each particle in the domain of P with the particle at the same relative position in the domain of Q
    ChVector3d shift = pos_Q - pos_P;
    std::vector<size_t> in_P;
    std::vector<size_t> in_Q;
    for (size_t i = 0; i < num_sph; i++) {
        if (Inside(pos0[i], abs_P))
            in_P.push_back(i);
        if (Inside(pos0[i], abs_Q))
            in_Q.push_back(i);
    }

    int num_pairs = 0;
    int num_pairs_face = 0;  // pairs within kernel reach of the +x face of Q
    double max_speed = 0;
    double max_diff = 0;
    double max_diff_face = 0;
    for (auto i : in_P) {
        ChVector3d target = pos0[i] + shift;
        auto it = std::find_if(in_Q.begin(), in_Q.end(), [&](size_t j) { return (pos0[j] - target).Length() < 1e-3 * spacing; });
        if (it == in_Q.end())
            continue;
        size_t j = *it;
        num_pairs++;
        double diff = (vel1[i] - vel1[j]).Length();
        max_speed = std::max(max_speed, std::max(vel1[i].Length(), vel1[j].Length()));
        max_diff = std::max(max_diff, diff);
        if (pos0[j].x() > abs_Q.max.x() - kernel_radius) {
            num_pairs_face++;
            max_diff_face = std::max(max_diff_face, diff);
        }
    }

    std::cout << "SPH particles: " << num_sph << std::endl;
    std::cout << "  in domain P:                      " << in_P.size() << std::endl;
    std::cout << "  in domain Q:                      " << in_Q.size() << std::endl;
    std::cout << "  P/Q pairs:                        " << num_pairs << std::endl;
    std::cout << "  pairs near the +x face of Q:      " << num_pairs_face << std::endl;
    std::cout << "  max speed:                        " << max_speed << std::endl;
    std::cout << "  max velocity difference:          " << max_diff << std::endl;
    std::cout << "  max difference near +x face of Q: " << max_diff_face << std::endl;

    Check("particles in domain P", !in_P.empty());
    Check("every particle in P has a partner in Q", num_pairs == (int)in_P.size() && in_P.size() == in_Q.size());
    Check("pairs near the +x face of Q", num_pairs_face > 0);
    Check("particles in the domains move", max_speed > 0);

    // The actual check: the halo of Q is intact, so P and Q evolve identically up to round-off. Measured: about 2e-7 of
    // the max speed with the halo intact, about 1e-2 with the +x halo of Q dropped.
    Check("velocities in domains P and Q agree", max_diff <= 1e-4 * max_speed);

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll active-domain halo checks passed" << std::endl;
    return 0;
}
