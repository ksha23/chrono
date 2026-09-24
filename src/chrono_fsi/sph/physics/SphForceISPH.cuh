// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Radu Serban
// =============================================================================

#ifndef CH_SPH_FORCE_ISPH_H
#define CH_SPH_FORCE_ISPH_H

#include "chrono_fsi/sph/physics/SphForce.cuh"
#include "chrono_fsi/sph/math/SphLinearSolver.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Convergence state of a Jacobi solve, kept on the device so that iterations need no host round trip.
struct JacobiStateISPH {
    int iteration;  ///< number of iterations performed
    int converged;  ///< nonzero once the stopping test is met; further iterations are then no-ops
    Real residual;  ///< maximum residual after the last iteration
};

/// Inter-particle force calculation for the implicit SPH method.
class SphForceISPH : public SphForce {
  public:
    /// Force class implemented using incompressible SPH method with implicit integrator.
    SphForceISPH(FsiDataManager& data_mgr,  ///< FSI data manager
                 bool verbose,              ///< verbose output
                 bool check_errors          ///< check errors
    );

    ~SphForceISPH();

    virtual void Initialize() override;

  private:
    std::shared_ptr<LinearSolver> myLinearSolver;

    thrust::device_vector<Real> _sumWij_inv;
    thrust::device_vector<Real> G_i;
    thrust::device_vector<Real> A_i;
    thrust::device_vector<Real> L_i;
    thrust::device_vector<Real> csrValLaplacian;
    thrust::device_vector<Real3> csrValGradient;
    thrust::device_vector<Real> csrValFunction;
    thrust::device_vector<Real> AMatrix;
    thrust::device_vector<Real3> Normals;
    thrust::device_vector<Real3> V_star_new;
    thrust::device_vector<Real3> V_star_old;
    thrust::device_vector<Real> q_new;
    thrust::device_vector<Real> q_old;
    thrust::device_vector<Real> b1Vector;
    thrust::device_vector<Real3> b3Vector;
    thrust::device_vector<Real> Residuals;
    thrust::device_vector<Real> ResidualsBlockMax;        ///< per-block maxima of Residuals
    thrust::device_vector<JacobiStateISPH> JacobiStateD;  ///< state of the current Jacobi solve
    thrust::device_vector<Real4> rhoPresMuD_old;
    thrust::device_vector<Real4> posRadD_old;
    thrust::device_vector<Real3> velMasD_old;

    size_t numAllMarkers;
    size_t NNZ;

    bool m_check_errors;

    void ForceSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real time, Real step) override;

    void PreProcessor(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, bool calcLaplacianOperator);

    /// Run Jacobi iterations for V* (vector3 = true) or the pressure (vector3 = false) while
    /// (residual > tol || iteration < 3) && iteration < LinearSolver_Max_Iter.
    /// The stopping test is evaluated on the device after every iteration; the host polls it only every
    /// LinearSolver_Check_Interval iterations, and iterations launched after convergence do nothing.
    void SolveJacobi(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, bool vector3, double tol, uint numBlocks, uint numThreads, int& iteration, Real& residual);
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
