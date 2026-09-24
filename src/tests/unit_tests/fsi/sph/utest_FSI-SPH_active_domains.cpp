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
// Unit test for SPH particle activity with several FSI bodies, each with its own
// active domain. Pins which particles are active, as computed on the device, to
// a host-side replica of the active AABBs.
//
// Setup: a CRM bed and five fixed rigid bodies above it. Four bodies have
// explicit active domains of different sizes, one of them offset from the body
// and one on a rotated body. The fifth body, at the center of the bed, has no
// active domain (inverted AABB) and must not activate anything. The layout
// leaves particles in three classes: inside an active domain, between the
// domains (inside their bounding box but in none of them), and outside the
// bounding box of all domains.
//
// Observable: an SPH particle that is not active is not integrated, so its
// position stays bitwise unchanged. Over a few steps, a particle must move if
// and only if it starts inside the active AABB of some body. Particles within a
// small distance of an active AABB face are left out of the comparison, since
// their classification depends on rounding.
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

struct BodySpec {
    ChVector3d pos;   // body position
    double angle;     // body rotation about the vertical axis
    bool has_domain;  // false: no active domain for this body
    ChAABB domain;    // active domain, expressed in the body frame
};

// Active AABB of a body in the absolute frame: the AABB of the rotated body-frame box, shifted by the body position.
ChAABB AbsoluteDomain(const BodySpec& b) {
    ChQuaterniond rot = QuatFromAngleZ(b.angle);
    ChAABB abs_aabb;
    for (int corner = 0; corner < 8; corner++) {
        ChVector3d p((corner & 1) ? b.domain.max.x() : b.domain.min.x(),  //
                     (corner & 2) ? b.domain.max.y() : b.domain.min.y(),  //
                     (corner & 4) ? b.domain.max.z() : b.domain.min.z());
        ChVector3d q = b.pos + rot.Rotate(p);
        abs_aabb.min = Vmin(abs_aabb.min, q);
        abs_aabb.max = Vmax(abs_aabb.max, q);
    }
    return abs_aabb;
}

bool Inside(const ChVector3d& p, const ChAABB& aabb) {
    return p.x() >= aabb.min.x() && p.x() <= aabb.max.x() &&  //
           p.y() >= aabb.min.y() && p.y() <= aabb.max.y() &&  //
           p.z() >= aabb.min.z() && p.z() <= aabb.max.z();
}

// Distance from p to the nearest face of the AABB, if p is within the face's extent in the other two directions.
double DistanceToFaces(const ChVector3d& p, const ChAABB& aabb) {
    double d = 1e30;
    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        if (p[j] < aabb.min[j] || p[j] > aabb.max[j] || p[k] < aabb.min[k] || p[k] > aabb.max[k])
            continue;
        d = std::min(d, std::min(std::abs(p[i] - aabb.min[i]), std::abs(p[i] - aabb.max[i])));
    }
    return d;
}

int main(int argc, char* argv[]) {
    double spacing = 0.02;
    double L = 0.8;
    double H = 0.1;
    double body_radius = 0.02;
    double dt = 1e-4;
    int num_steps = 3;

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
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Fixed bodies hovering above the bed. Domain faces are offset from the particle layers.
    double z = H + 0.03;
    std::vector<BodySpec> specs = {
        {ChVector3d(-0.24, -0.24, z), 0.0, true, ChAABB(ChVector3d(-0.061, -0.061, -0.101), ChVector3d(0.061, 0.061, 0.101))},
        {ChVector3d(+0.24, -0.24, z), 0.0, true, ChAABB(ChVector3d(-0.051, -0.041, -0.081), ChVector3d(0.089, 0.041, 0.021))},
        {ChVector3d(-0.20, +0.24, z), CH_PI / 6, true, ChAABB(ChVector3d(-0.061, -0.031, -0.101), ChVector3d(0.061, 0.031, 0.101))},
        {ChVector3d(+0.24, +0.24, z), 0.0, true, ChAABB(ChVector3d(-0.101, -0.041, -0.061), ChVector3d(0.101, 0.041, 0.061))},
        {ChVector3d(0.0, 0.0, z), 0.0, false, ChAABB()},
    };

    for (const auto& s : specs) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(s.pos);
        body->SetRot(QuatFromAngleZ(s.angle));
        body->SetMass(1.0);
        body->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
        body->SetFixed(true);
        sysMBS.AddBody(body);
        fsi.AddRigidBodySphere(body, ChVector3d(0, 0, 0), body_radius, false);
        if (s.has_domain)
            fsi.SetActiveDomainBody(body, s.domain);
    }

    // Bed with an open top
    fsi.Construct(ChVector3d(L, L, H), ChVector3d(0, 0, 0), BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS | BoxSide::Y_NEG | BoxSide::Y_POS);

    fsi.Initialize();

    size_t num_sph = fsi.GetNumSPHParticles();
    auto pos0 = sysSPH->GetParticlePositions();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    auto pos1 = sysSPH->GetParticlePositions();

    // Host replica of the active AABBs (absolute frame) and their bounding box
    std::vector<ChAABB> domains;
    ChAABB bounds;
    for (const auto& s : specs) {
        if (!s.has_domain)
            continue;
        domains.push_back(AbsoluteDomain(s));
        bounds += domains.back();
    }

    double margin = 1e-3 * spacing;
    int num_inside = 0;         // inside some active AABB
    int num_between = 0;        // inside the bounding box of the active AABBs, but in none of them
    int num_outside = 0;        // outside the bounding box of the active AABBs
    int num_skipped = 0;        // too close to an active AABB face to classify
    int num_moved = 0;          // position changed
    int num_frozen_inside = 0;  // inside an active AABB, but not moved
    int num_moved_outside = 0;  // outside every active AABB, but moved

    for (size_t i = 0; i < num_sph; i++) {
        const ChVector3d& p = pos0[i];
        bool moved = pos1[i].x() != p.x() || pos1[i].y() != p.y() || pos1[i].z() != p.z();
        if (moved)
            num_moved++;

        bool near_face = false;
        bool inside = false;
        for (const auto& d : domains) {
            near_face |= DistanceToFaces(p, d) < margin;
            inside |= Inside(p, d);
        }
        if (near_face) {
            num_skipped++;
            continue;
        }

        if (inside) {
            num_inside++;
            if (!moved)
                num_frozen_inside++;
        } else {
            if (Inside(p, bounds))
                num_between++;
            else
                num_outside++;
            if (moved)
                num_moved_outside++;
        }
    }

    std::cout << "SPH particles: " << num_sph << std::endl;
    std::cout << "  inside an active domain:            " << num_inside << std::endl;
    std::cout << "  between active domains:             " << num_between << std::endl;
    std::cout << "  outside all active domains:         " << num_outside << std::endl;
    std::cout << "  skipped (near a domain face):       " << num_skipped << std::endl;
    std::cout << "  moved:                              " << num_moved << std::endl;
    std::cout << "  inside, not moved:                  " << num_frozen_inside << std::endl;
    std::cout << "  outside every domain, moved:        " << num_moved_outside << std::endl;

    // Guards against the checks below passing on a degenerate layout
    Check("particles inside active domains", num_inside > 0);
    Check("particles between active domains", num_between > 0);
    Check("particles outside all active domains", num_outside > 0);
    Check("few particles too close to a face to classify", num_skipped < num_inside / 10);

    // The actual checks: activity on the device matches the host replica of the active domains
    Check("every particle inside an active domain is active", num_frozen_inside == 0);
    Check("no particle outside every active domain is active", num_moved_outside == 0);

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll active-domain checks passed" << std::endl;
    return 0;
}
