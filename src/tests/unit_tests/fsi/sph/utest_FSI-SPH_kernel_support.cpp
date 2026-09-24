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
// Unit test for the SPH neighbor search radius.
//
// The neighbor list must contain every particle within the kernel support radius h_multiplier * h
// (2h for the cubic spline and Wendland kernels, 3h for the quintic spline). The test builds a fully
// periodic, uniform particle lattice at rest with a small sinusoidal density (and hence pressure)
// perturbation along x and no gravity. The particles start at rest, so after one very small step the
// particle velocity divided by the step size is the discrete pressure gradient term -grad(p)/rho. Its
// amplitude, relative to the analytic value obtained from the particle pressures, is compared with the
// same ratio evaluated on the host as a lattice sum over the full kernel support. If the neighbor list
// is truncated inside the kernel support, the discrete gradient loses the contribution of the missing
// pairs and its amplitude is too small (about 13% for the quintic spline truncated at 2h).
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Ratio between the SPH and the analytic pressure-gradient acceleration amplitudes (simulated and host
// reference), plus the largest transverse (y, z) acceleration relative to the analytic amplitude.
struct GradientResult {
    double ratio;
    double ratio_ref;
    double transverse;
    size_t num_particles;
};

// Reference kernel gradients (same definitions as the Chrono::FSI-SPH kernels), returned as the factor
// f(q) such that grad W(r) = f(q) * r, with q = |r| / h.
static double GradFactor(KernelType kernel, double q, double h) {
    double h5 = std::pow(h, 5);
    switch (kernel) {
        case KernelType::CUBIC_SPLINE: {
            double beta = 3 / (4 * CH_PI * h5);
            if (q < 1)
                return beta * (3 * q - 4);
            if (q < 2)
                return beta * (4 - q - 4 / q);
            return 0;
        }
        case KernelType::QUINTIC_SPLINE: {
            double beta = -5 / (120 * CH_PI * h5);
            auto p4 = [](double x) { return x > 0 ? x * x * x * x : 0.0; };
            if (q < 3)
                return (beta / q) * (p4(3 - q) - 6 * p4(2 - q) + 15 * p4(1 - q));
            return 0;
        }
        case KernelType::WENDLAND: {
            double beta = -210 / (256 * CH_PI * h5);
            if (q < 2)
                return beta * (2 - q) * (2 - q) * (2 - q);
            return 0;
        }
        default:
            return 0;
    }
}

// Host lattice sum of the SPH gradient of sin(k x) over the full kernel support, relative to the analytic
// gradient k cos(k x). Only the p_j term of the symmetric pressure gradient contributes on a uniform lattice.
static double ReferenceRatio(KernelType kernel, double spacing, double h, double support, double k, double volume) {
    int n = (int)std::ceil(support / spacing);
    double sum = 0;
    for (int i = -n; i <= n; i++)
        for (int j = -n; j <= n; j++)
            for (int l = -n; l <= n; l++) {
                ChVector3d s(i * spacing, j * spacing, l * spacing);
                double r = s.Length();
                if (r == 0 || r >= support)
                    continue;
                sum += std::sin(k * s.x()) * GradFactor(kernel, r / h, h) * s.x();
            }
    return -volume * sum / k;
}

static GradientResult PressureGradientRatio(KernelType kernel) {
    const double spacing = 0.01;
    const int nx = 40;  // wavelength of the perturbation = periodic length along x
    const int ny = 12;  // periodic length along y and z must span at least 3 grid cells for all kernels
    const int nz = 12;
    const double Lx = nx * spacing;
    const double Ly = ny * spacing;
    const double Lz = nz * spacing;
    const double eps = 1e-3;  // relative density perturbation
    const double k = CH_2PI / Lx;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetVerbose(false);
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1e-3;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.kernel_type = kernel;
    sph_params.max_velocity = 1.0;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.use_delta_sph = false;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = 1;
    sysSPH.SetSPHParameters(sph_params);

    const double dt = 1e-5;
    sysFSI.SetStepSizeCFD(dt);
    sysFSI.SetStepsizeMBD(dt);

    sysSPH.SetComputationalDomain(ChAABB(ChVector3d(0, 0, 0), ChVector3d(Lx, Ly, Lz)), BC_ALL_PERIODIC);

    const double rho0 = sysSPH.GetDensity();
    const double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            for (int iz = 0; iz < nz; iz++) {
                ChVector3d pos((ix + 0.5) * spacing, (iy + 0.5) * spacing, (iz + 0.5) * spacing);
                double rho = rho0 * (1 + eps * std::sin(k * pos.x()));
                double p = c2 * (rho - rho0);
                sysSPH.AddSPHParticle(pos, rho, p, sysSPH.GetViscosity());
            }
        }
    }

    sysFSI.Initialize();
    sysFSI.DoStepDynamics(dt);

    size_t n = sysSPH.GetNumFluidMarkers();
    auto pos = sysSPH.GetParticlePositions();
    auto props = sysSPH.GetParticleFluidProperties();  // (rho, p, mu)
    auto vel = sysSPH.GetParticleVelocities();

    // Least-squares fit of the reported pressures p = P0 + A sin(kx) + B cos(kx)
    double sss = 0, scc = 0, ssc = 0, ss = 0, sc = 0, sps = 0, spc = 0, sp = 0;
    for (size_t i = 0; i < n; i++) {
        double s = std::sin(k * pos[i].x());
        double c = std::cos(k * pos[i].x());
        double p = props[i].y();
        sss += s * s;
        scc += c * c;
        ssc += s * c;
        ss += s;
        sc += c;
        sps += p * s;
        spc += p * c;
        sp += p;
    }
    // On a uniform periodic lattice the sine and cosine columns are orthogonal and have zero mean
    double A = sps / sss;
    double B = spc / scc;

    // Analytic acceleration -1/rho dp/dx and its projection on the SPH values
    double num = 0, den = 0, amax = 0, tmax = 0;
    for (size_t i = 0; i < n; i++) {
        double x = pos[i].x();
        double a_ref = -(k / props[i].x()) * (A * std::cos(k * x) - B * std::sin(k * x));
        ChVector3d acc = vel[i] / dt;
        num += acc.x() * a_ref;
        den += a_ref * a_ref;
        amax = std::max(amax, std::abs(a_ref));
        tmax = std::max(tmax, std::max(std::abs(acc.y()), std::abs(acc.z())));
    }

    // Reference ratio over the full kernel support; the SPH particle volume is m / rho0
    double h = sysSPH.GetKernelLength();
    double support = (kernel == KernelType::QUINTIC_SPLINE ? 3 : 2) * h;
    double volume = sysSPH.GetParticleMass() / rho0;

    GradientResult r;
    r.ratio = num / den;
    r.ratio_ref = ReferenceRatio(kernel, spacing, h, support, k, volume);
    r.transverse = tmax / amax;
    r.num_particles = n;

    std::cout << "  kernel " << static_cast<int>(kernel) << ": " << n << " particles, pressure amplitude " << std::sqrt(A * A + B * B)
              << " Pa, SPH / analytic gradient amplitude = " << r.ratio << " (full-support lattice sum " << r.ratio_ref << "), max transverse / axial = " << r.transverse
              << std::endl;
    return r;
}

// The simulated gradient must match the full-support lattice sum. The remaining difference comes from the
// single-precision device arithmetic and the small density variation (about 0.1%).
const double ratio_tolerance = 1e-3;

TEST(SPH_kernel_support, pressure_gradient_cubic) {
    auto r = PressureGradientRatio(KernelType::CUBIC_SPLINE);
    ASSERT_EQ(r.num_particles, 40u * 12u * 12u);
    EXPECT_NEAR(r.ratio, r.ratio_ref, ratio_tolerance);
    EXPECT_LT(r.transverse, 1e-3);
}

TEST(SPH_kernel_support, pressure_gradient_wendland) {
    auto r = PressureGradientRatio(KernelType::WENDLAND);
    ASSERT_EQ(r.num_particles, 40u * 12u * 12u);
    EXPECT_NEAR(r.ratio, r.ratio_ref, ratio_tolerance);
    EXPECT_LT(r.transverse, 1e-3);
}

TEST(SPH_kernel_support, pressure_gradient_quintic) {
    auto r = PressureGradientRatio(KernelType::QUINTIC_SPLINE);
    ASSERT_EQ(r.num_particles, 40u * 12u * 12u);
    EXPECT_NEAR(r.ratio, r.ratio_ref, ratio_tolerance);
    EXPECT_LT(r.transverse, 1e-3);
}
