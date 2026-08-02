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

#ifndef CH_SOLVER_AUTO_H
#define CH_SOLVER_AUTO_H

#include <memory>

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChSolver.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// A solver that dispatches each step to either a sparse direct solver or an iterative
/// VI solver, depending on the problem actually presented at that step.
///
/// ### Why
///
/// The iterative VI solvers (PSOR, PJacobi, BB, APGD) solve a variational inequality by
/// projected gradient iteration. When every active constraint is bilateral, there is no
/// inequality left to satisfy and the problem is a plain linear system - which those
/// methods then attack with a first-order method. On a serially coupled model (a chain,
/// a linkage, a driveline) the Schur complement of such a system is badly conditioned
/// and a first-order method converges far too slowly to be usable at any affordable
/// iteration count, while a sparse direct solver returns the exact answer and is
/// typically *faster* as well.
///
/// Conversely, when contacts or other unilateral constraints are present a direct linear
/// solver is simply not applicable: it would treat every contact as bilateral glue, so
/// bodies would stick to surfaces instead of separating from them.
///
/// ### The rule
///
/// The direct solver is used for a step only when **all** of the following hold:
///  - every active constraint is in ChConstraint::Mode::LOCK (bilateral). Contacts use
///    Mode::FRICTION and joint limits use Mode::UNILATERAL; either disqualifies the step.
///    Note that ChConstraint::IsUnilateral() is *not* a sufficient test, as it returns
///    false for Mode::FRICTION contact constraints.
///  - the system descriptor reports no stiffness/damping (KRM) blocks requiring the
///    fallback's Schur complement form, or the fallback cannot be used at all.
///  - the factorization is not too dense: see SetMaxFactorFillRatio(). Fill-in, not
///    problem size, is what makes a direct solve expensive. A banded system (chain,
///    tree, most mechanisms and vehicles) stays cheap to very large size; a 3-D
///    connected system fills in rapidly and is better left to the iterative solver.
///  - the factorization has not previously failed on this system (a rank-deficient
///    constraint set, e.g. redundant constraints in a closed kinematic loop, makes a
///    plain LU factorization unusable).
///
/// The decision is taken in Setup() and can therefore change during a simulation, for
/// example when a model that started free of contact acquires one.
///
/// ### Inspecting and overriding
///
/// GetActiveSolver() reports which solver handled the last step and WhyIterative()
/// explains the most recent decision. SetVerbose(true) logs every change of regime.
/// Both sub-solvers are accessible and replaceable via GetDirectSolver() /
/// GetIterativeSolver() / SetDirectSolver() / SetIterativeSolver(). To bypass the
/// mechanism entirely, install a concrete solver with ChSystem::SetSolver.
///
/// AsIterative() always returns the iterative sub-solver, so existing code of the form
/// `sys.GetSolver()->AsIterative()->SetMaxIterations(n)` keeps configuring the
/// iterative fallback as before.
class ChApi ChSolverAuto : public ChSolver {
  public:
    /// Reason the iterative solver was selected for the most recent step.
    enum class Reason {
        DIRECT,             ///< the direct solver was used
        UNILATERAL,         ///< at least one active constraint is not bilateral
        UNSUPPORTED,        ///< the direct solver cannot represent this problem
        TOO_MUCH_FILL_IN,   ///< the factorization is denser than the allowed budget
        FACTORIZATION_FAIL  ///< a previous factorization failed (e.g. singular matrix)
    };

    ChSolverAuto();
    ~ChSolverAuto() {}

    virtual Type GetType() const override { return Type::AUTO; }

    /// Reported as iterative: the iterative sub-solver is always available for
    /// configuration, whichever solver actually ran the last step.
    virtual bool IsIterative() const override { return true; }
    virtual bool IsDirect() const override { return false; }

    /// Return the iterative sub-solver, so that existing calling code that configures
    /// iteration counts and tolerances continues to work unchanged.
    virtual ChIterativeSolver* AsIterative() override { return m_iterative.get(); }

    /// Conservatively true: which solver will run is not known until Setup() has
    /// inspected the descriptor, and the iterative solver needs the matrix.
    virtual bool SolveRequiresMatrix() const override { return true; }

    virtual bool Setup(ChSystemDescriptor& sysd, bool analyze) override;
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Replace the direct sub-solver (default: ChSolverSparseLU with locked sparsity).
    void SetDirectSolver(std::shared_ptr<ChDirectSolverLS> solver);

    /// Replace the iterative sub-solver (default: ChSolverBB).
    void SetIterativeSolver(std::shared_ptr<ChIterativeSolverVI> solver);

    std::shared_ptr<ChDirectSolverLS> GetDirectSolver() const { return m_direct; }
    std::shared_ptr<ChIterativeSolverVI> GetIterativeSolver() const { return m_iterative; }

    /// Largest allowed ratio nnz(L+U) / n before the direct solver is abandoned as too
    /// expensive (default: 40). This is a measure of fill-in per row of the factorized
    /// KKT matrix. Chain-like and tree-like models stay well under it at any size;
    /// densely connected models exceed it quickly. Set to 0 to disable the check.
    void SetMaxFactorFillRatio(double ratio) { m_max_fill_ratio = ratio; }
    double GetMaxFactorFillRatio() const { return m_max_fill_ratio; }

    /// Never use the direct solver above this problem dimension, regardless of fill-in
    /// (default: 50000). This bounds the memory, and the cost of the one-time probe
    /// factorization used to measure fill-in, that the direct path may claim.
    void SetMaxDirectDim(unsigned int dim) { m_max_direct_dim = dim; }
    unsigned int GetMaxDirectDim() const { return m_max_direct_dim; }

    /// Force one solver for all steps, disabling the automatic choice.
    /// Useful to reproduce results obtained before this solver existed.
    void ForceIterative(bool val) { m_force_iterative = val; }
    void ForceDirect(bool val) { m_force_direct = val; }

    /// Return the solver that handled the last step.
    ChSolver* GetActiveSolver() const;

    /// Return true if the last step was handled by the direct solver.
    bool UsedDirectSolver() const { return m_used_direct; }

    /// Return the reason for the most recent decision.
    Reason GetReason() const { return m_reason; }

    /// Return a human-readable form of the most recent decision.
    std::string WhyIterative() const;

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    /// Return true if every active constraint in the descriptor is bilateral.
    static bool AllConstraintsBilateral(ChSystemDescriptor& sysd);

    std::shared_ptr<ChDirectSolverLS> m_direct;
    std::shared_ptr<ChIterativeSolverVI> m_iterative;

    bool m_used_direct;        ///< did the direct solver handle the last step?
    bool m_direct_disabled;    ///< has the direct solver been ruled out for good?
    bool m_force_iterative;    ///< user override
    bool m_force_direct;       ///< user override
    Reason m_reason;           ///< reason for the most recent decision
    double m_max_fill_ratio;   ///< fill-in budget
    unsigned int m_max_direct_dim;  ///< memory guard
    unsigned int m_last_n_var;      ///< problem shape at the previous direct setup
    unsigned int m_last_n_con;      ///< problem shape at the previous direct setup
};

/// @} chrono_solver

}  // end namespace chrono

#endif
