// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================

#include "chrono_multicore/solver/ChSolverMulticore.h"

using namespace chrono;

ChSchurProduct::ChSchurProduct() {
    data_manager = 0;
}

void ChSchurProduct::Setup(ChMulticoreDataManager* data_container_) {
    data_manager = data_container_;

    const SparseMatrixType& D_T = data_manager->host_data.D_T;
    const SparseMatrixType& M_invD = data_manager->host_data.M_invD;

    const uint num_rigid_dof = data_manager->num_rigid_bodies * 6;
    const uint num_shaft_dof = data_manager->num_shafts;
    const uint num_motor_dof = data_manager->num_motors;
    const uint num_uni = data_manager->num_unilaterals;
    const uint num_bil = data_manager->num_bilaterals;
    const uint num_r_c = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;
    const SolverMode solver_mode = data_manager->settings.solver.solver_mode;

    const uint bil_dof = num_rigid_dof + num_shaft_dof + num_motor_dof;

    if (num_r_c > 0 && num_uni > 0) {
        m_D_n_T = D_T.middleRows(0, num_r_c).leftCols(bil_dof);
        m_M_invD_n = M_invD.middleCols(0, num_r_c).topRows(bil_dof);

        if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
            m_D_t_T = D_T.middleRows(num_r_c, 2 * num_r_c).leftCols(bil_dof);
            m_M_invD_t = M_invD.middleCols(num_r_c, 2 * num_r_c).topRows(bil_dof);
        }
        if (solver_mode == SolverMode::SPINNING) {
            m_D_s_T = D_T.middleRows(3 * num_r_c, 3 * num_r_c).leftCols(bil_dof);
            m_M_invD_s = M_invD.middleCols(3 * num_r_c, 3 * num_r_c).topRows(bil_dof);
        }
    }
    if (num_bil > 0) {
        m_D_b_T = D_T.middleRows(num_uni, num_bil).leftCols(bil_dof);
        m_M_invD_b = M_invD.middleCols(num_uni, num_bil).topRows(bil_dof);
    }
}

// Below this number of nonzeros the product runs serially (same threshold as Eigen's own parallel product).
static const Eigen::Index spmv_parallel_min_nnz = 20000;

void ChSchurProduct::SpMV(const SparseMatrixType& A, Eigen::Ref<const VectorType> x, Eigen::Ref<VectorType> y, bool accumulate) {
    // Chrono_multicore is built with EIGEN_DONT_PARALLELIZE (Eigen must not spawn threads inside Chrono's own OpenMP
    // regions), so an Eigen product would run serially here. The Schur product is called outside any parallel region.
    assert(x.size() == A.cols() && y.size() == A.rows());

    using Index = SparseMatrixType::StorageIndex;
    const Index num_rows = (Index)A.rows();
    const Index* outer = A.outerIndexPtr();
    const Index* inner = A.innerIndexPtr();
    const Index* inner_nnz = A.innerNonZeroPtr();  // null for a compressed matrix
    const real* val = A.valuePtr();
    const real* xp = x.data();
    real* yp = y.data();

    // Same per-row operation order as Eigen's serial row-major product, so results are bitwise identical to it
#pragma omp parallel for schedule(static) if (A.nonZeros() > spmv_parallel_min_nnz)
    for (Index i = 0; i < num_rows; i++) {
        const Index end = inner_nnz ? outer[i] + inner_nnz[i] : outer[i + 1];
        real tmp = 0;
        for (Index k = outer[i]; k < end; k++)
            tmp += val[k] * xp[inner[k]];
        yp[i] = (accumulate ? yp[i] : real(0)) + tmp;
    }
}

void ChSchurProduct::operator()(const VectorType& x, VectorType& output) {
    data_manager->system_timer.start("SchurProduct");

    const VectorType& E = data_manager->host_data.E;

    uint num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    output.setZero();

    const SparseMatrixType& D_T = data_manager->host_data.D_T;
    const SparseMatrixType& Nschur = data_manager->host_data.Nschur;

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        if (data_manager->settings.solver.compute_N) {
            SpMV(Nschur, x, output);
            output += E.cwiseProduct(x);
        } else {
            m_tmp.resize(data_manager->host_data.M_invD.rows());
            SpMV(data_manager->host_data.M_invD, x, m_tmp);
            SpMV(D_T, m_tmp, output);
            output += E.cwiseProduct(x);
        }

    } else {
        const uint bil_dof = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_motors;

        SubVectorType o_b = output.segment(num_unilaterals, num_bilaterals);
        ConstSubVectorType x_b = x.segment(num_unilaterals, num_bilaterals);
        ConstSubVectorType E_b = E.segment(num_unilaterals, num_bilaterals);

        SubVectorType o_n = output.segment(0, num_rigid_contacts);
        ConstSubVectorType x_n = x.segment(0, num_rigid_contacts);
        ConstSubVectorType E_n = E.segment(0, num_rigid_contacts);

        switch (data_manager->settings.solver.local_solver_mode) {
            case SolverMode::BILATERAL: {
                m_tmp.resize(m_M_invD_b.rows());
                SpMV(m_M_invD_b, x_b, m_tmp);
                SpMV(m_D_b_T, m_tmp, o_b);
                o_b += E_b.cwiseProduct(x_b);
            } break;

            case SolverMode::NORMAL: {
                m_tmp.setZero(bil_dof);
                if (num_rigid_contacts > 0)
                    SpMV(m_M_invD_n, x_n, m_tmp, true);
                if (num_bilaterals > 0) {
                    SpMV(m_M_invD_b, x_b, m_tmp, true);
                    SpMV(m_D_b_T, m_tmp, o_b);
                    o_b += E_b.cwiseProduct(x_b);
                }
                if (num_rigid_contacts > 0) {
                    SpMV(m_D_n_T, m_tmp, o_n);
                    o_n += E_n.cwiseProduct(x_n);
                }
            } break;

            case SolverMode::SLIDING: {
                SubVectorType o_t = output.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = x.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = E.segment(num_rigid_contacts, num_rigid_contacts * 2);

                m_tmp.setZero(bil_dof);
                if (num_rigid_contacts > 0) {
                    SpMV(m_M_invD_n, x_n, m_tmp, true);
                    SpMV(m_M_invD_t, x_t, m_tmp, true);
                }
                if (num_bilaterals > 0) {
                    SpMV(m_M_invD_b, x_b, m_tmp, true);
                    SpMV(m_D_b_T, m_tmp, o_b);
                    o_b += E_b.cwiseProduct(x_b);
                }
                if (num_rigid_contacts > 0) {
                    SpMV(m_D_n_T, m_tmp, o_n);
                    o_n += E_n.cwiseProduct(x_n);
                    SpMV(m_D_t_T, m_tmp, o_t);
                    o_t += E_t.cwiseProduct(x_t);
                }

            } break;

            case SolverMode::SPINNING: {
                SubVectorType o_t = output.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = x.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = E.segment(num_rigid_contacts, num_rigid_contacts * 2);

                SubVectorType o_s = output.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType x_s = x.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType E_s = E.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);

                m_tmp.setZero(bil_dof);
                if (num_rigid_contacts > 0) {
                    SpMV(m_M_invD_n, x_n, m_tmp, true);
                    SpMV(m_M_invD_t, x_t, m_tmp, true);
                    SpMV(m_M_invD_s, x_s, m_tmp, true);
                }
                if (num_bilaterals > 0) {
                    SpMV(m_M_invD_b, x_b, m_tmp, true);
                    SpMV(m_D_b_T, m_tmp, o_b);
                    o_b += E_b.cwiseProduct(x_b);
                }
                if (num_rigid_contacts > 0) {
                    SpMV(m_D_n_T, m_tmp, o_n);
                    o_n += E_n.cwiseProduct(x_n);
                    SpMV(m_D_t_T, m_tmp, o_t);
                    o_t += E_t.cwiseProduct(x_t);
                    SpMV(m_D_s_T, m_tmp, o_s);
                    o_s += E_s.cwiseProduct(x_s);
                }

            } break;
        }
    }
    data_manager->system_timer.stop("SchurProduct");
}

void ChSchurProductBilateral::Setup(ChMulticoreDataManager* data_container_) {
    ChSchurProduct::Setup(data_container_);
    if (data_manager->num_bilaterals == 0) {
        return;
    }
    NschurB = _DBT_ * _MINVDB_;
}

void ChSchurProductBilateral::operator()(const VectorType& x, VectorType& output) {
    output.noalias() = NschurB * x;
}
