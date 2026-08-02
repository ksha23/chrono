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
// Authors: Radu Serban
// =============================================================================

#include <iostream>

#include "chrono/solver/ChSolverAuto.h"
#include "chrono/solver/ChSolverBB.h"

namespace chrono {

CH_FACTORY_REGISTER(ChSolverAuto)

ChSolverAuto::ChSolverAuto()
    : m_used_direct(false),
      m_direct_disabled(false),
      m_force_iterative(false),
      m_force_direct(false),
      m_reason(Reason::DIRECT),
      m_max_fill_ratio(40.0),
      m_max_direct_dim(50000),
      m_last_n_var(0),
      m_last_n_con(0) {
    auto direct = chrono_types::make_shared<ChSolverSparseLU>();
    // Models handled by the direct path have a fixed topology by construction (no
    // contacts), so the sparsity pattern need only be learned once.
    direct->LockSparsityPattern(true);
    m_direct = direct;

    m_iterative = chrono_types::make_shared<ChSolverBB>();
}

void ChSolverAuto::SetDirectSolver(std::shared_ptr<ChDirectSolverLS> solver) {
    m_direct = solver;
    m_direct_disabled = false;
}

void ChSolverAuto::SetIterativeSolver(std::shared_ptr<ChIterativeSolverVI> solver) {
    m_iterative = solver;
}

ChSolver* ChSolverAuto::GetActiveSolver() const {
    return m_used_direct ? static_cast<ChSolver*>(m_direct.get()) : static_cast<ChSolver*>(m_iterative.get());
}

bool ChSolverAuto::AllConstraintsBilateral(ChSystemDescriptor& sysd) {
    // Only ChConstraint::Mode::LOCK is a genuine equality constraint.
    //
    // Do NOT use ChConstraint::IsUnilateral() here: it tests for Mode::UNILATERAL only,
    // while NSC contact constraints (normal, tangential and rolling alike) are created
    // in Mode::FRICTION and would be misreported as bilateral.
    for (auto c : sysd.GetConstraints()) {
        if (!c->IsActive())
            continue;
        if (c->GetMode() != ChConstraint::Mode::LOCK)
            return false;
    }
    return true;
}

bool ChSolverAuto::Setup(ChSystemDescriptor& sysd, bool analyze) {
    bool was_direct = m_used_direct;

    // ---------------------------------------------------------------- decide
    bool use_direct;
    if (m_force_direct) {
        use_direct = true;
        m_reason = Reason::DIRECT;
    } else if (m_force_iterative) {
        use_direct = false;
        m_reason = Reason::UNSUPPORTED;
    } else if (m_direct_disabled) {
        use_direct = false;
        m_reason = Reason::FACTORIZATION_FAIL;
    } else if (!AllConstraintsBilateral(sysd)) {
        use_direct = false;
        m_reason = Reason::UNILATERAL;
    } else if (sysd.CountActiveVariables() + sysd.CountActiveConstraints() > m_max_direct_dim) {
        use_direct = false;
        m_reason = Reason::TOO_MUCH_FILL_IN;
    } else {
        use_direct = true;
        m_reason = Reason::DIRECT;
    }

    // ------------------------------------------------------------- direct try
    if (use_direct) {
        // A structural change relative to the previous step must be analyzed afresh.
        bool do_analyze = analyze || !was_direct;

        // The direct sub-solver runs with a locked sparsity pattern, which is only safe
        // while the topology is fixed. The `analyze` flag cannot be used to detect a
        // change: EULER_IMPLICIT_LINEARIZED, the default integrator, passes it on every
        // step, and re-running the pattern learner every step costs more than the
        // factorization itself (197 -> 292 us/step on an HMMWV lane change). Track the
        // problem shape instead and re-learn only when it actually moves.
        unsigned int n_var = sysd.CountActiveVariables();
        unsigned int n_con = sysd.CountActiveConstraints();
        if (!was_direct || n_var != m_last_n_var || n_con != m_last_n_con)
            m_direct->ForceSparsityPatternUpdate();
        m_last_n_var = n_var;
        m_last_n_con = n_con;

        bool ok = m_direct->Setup(sysd, do_analyze);

        if (!ok) {
            // A failed factorization normally means a rank-deficient constraint set,
            // for instance redundant constraints in a closed kinematic loop. This will
            // not fix itself, so stop trying and hand the problem to the iterative
            // solver, which tolerates redundancy.
            m_direct_disabled = true;
            use_direct = false;
            m_reason = Reason::FACTORIZATION_FAIL;
            std::cerr << "[ChSolverAuto] direct factorization failed; falling back to the "
                      << "iterative solver for the remainder of the simulation." << std::endl;
        } else if (m_max_fill_ratio > 0) {
            // Fill-in, not problem size, is what makes a sparse factorization expensive.
            // Check it once the factors actually exist.
            unsigned int nnz = m_direct->GetNumFactorNonzeros();
            unsigned int n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
            if (nnz > 0 && n > 0 && double(nnz) / double(n) > m_max_fill_ratio) {
                m_direct_disabled = true;
                use_direct = false;
                m_reason = Reason::TOO_MUCH_FILL_IN;
                if (verbose) {
                    std::cout << "[ChSolverAuto] factorization fill ratio " << double(nnz) / double(n) << " exceeds "
                              << m_max_fill_ratio << "; switching to the iterative solver." << std::endl;
                }
            }
        }
    }

    // ---------------------------------------------------------- iterative try
    if (!use_direct) {
        if (!m_iterative) {
            std::cerr << "[ChSolverAuto] no iterative fallback available." << std::endl;
            return false;
        }
        if (!sysd.SupportsSchurComplement() && m_iterative->GetType() != ChSolver::Type::CUSTOM) {
            // The Schur-complement solvers cannot represent stiffness/damping blocks.
            // If the direct solver was ruled out for applicability reasons there is
            // nothing left to try, so report the failure rather than throwing later.
            std::cerr << "[ChSolverAuto] the problem has stiffness/damping blocks and unilateral "
                      << "constraints; no built-in solver handles this combination. "
                      << "Install a suitable solver explicitly with ChSystem::SetSolver." << std::endl;
            return false;
        }
        m_iterative->Setup(sysd, analyze);
    }

    if (verbose && use_direct != was_direct) {
        std::cout << "[ChSolverAuto] switching to the " << (use_direct ? "direct" : "iterative")
                  << " solver (" << WhyIterative() << ")" << std::endl;
    }

    m_used_direct = use_direct;
    return true;
}

double ChSolverAuto::Solve(ChSystemDescriptor& sysd) {
    return m_used_direct ? m_direct->Solve(sysd) : m_iterative->Solve(sysd);
}

std::string ChSolverAuto::WhyIterative() const {
    switch (m_reason) {
        case Reason::DIRECT:
            return "all constraints bilateral, factorization affordable";
        case Reason::UNILATERAL:
            return "unilateral or frictional constraints present";
        case Reason::UNSUPPORTED:
            return "direct solver not applicable or disabled by the user";
        case Reason::TOO_MUCH_FILL_IN:
            return "factorization too dense";
        case Reason::FACTORIZATION_FAIL:
            return "factorization failed (constraint set likely rank deficient)";
    }
    return "unknown";
}

void ChSolverAuto::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChSolverAuto>();
    ChSolver::ArchiveOut(archive_out);
    archive_out << CHNVP(m_max_fill_ratio);
    archive_out << CHNVP(m_max_direct_dim);
    archive_out << CHNVP(m_force_iterative);
    archive_out << CHNVP(m_force_direct);
    archive_out << CHNVP(m_iterative);
}

void ChSolverAuto::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChSolverAuto>();
    ChSolver::ArchiveIn(archive_in);
    archive_in >> CHNVP(m_max_fill_ratio);
    archive_in >> CHNVP(m_max_direct_dim);
    archive_in >> CHNVP(m_force_iterative);
    archive_in >> CHNVP(m_force_direct);
    archive_in >> CHNVP(m_iterative);
}

}  // end namespace chrono
