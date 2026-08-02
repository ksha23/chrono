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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <limits>

#include "chrono/solver/ChSolverBB.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverBB)

// Default number of backtracking steps allowed in the non-monotone line search.
//
// Zero, i.e. the line search is off: every spectral step is accepted as-is.
//
// The line search code below was dead from the outset (the acceptance threshold was
// seeded with +10e29, which no finite objective can exceed), so no released version of
// Chrono has ever taken a backtracking step and every published BB result was obtained
// with the plain spectral step. The code is repaired rather than deleted, but leaving
// it enabled by default would silently change every existing BB result. Measurements on
// serial chains and 3-D lattices showed the repaired search firing several hundred times
// per run while leaving trajectory accuracy unchanged or marginally worse, so there is
// no evidence to justify that change. Enable it explicitly with
// SetMaxStepsArmijoBacktrace() if your problem benefits.
static const int CH_BB_DEFAULT_ARMIJO_BACKTRACKS = 0;

ChSolverBB::ChSolverBB()
    : n_armijo(10),
      max_armijo_backtrace(CH_BB_DEFAULT_ARMIJO_BACKTRACKS),
      n_armijo_backtracks(0),
      lastgoodres(1e30) {}

double ChSolverBB::Solve(ChSystemDescriptor& sysd) {
    if (!sysd.SupportsSchurComplement()) {
        std::cerr << "\n\nChSolverBB: Can NOT use Barzilai-Borwein solver if\n"
                  << " - there are stiffness or damping matrices, or\n "
                  << " - no inverse mass matrix was provided" << std::endl;
        throw std::runtime_error("ChSolverBB: System descriptor does not support Schur complement-based solvers.");
    }

    // Tuning of the spectral gradient search
    double a_min = 1e-13;
    double a_max = 1e13;
    double sigma_min = 0.1;
    double sigma_max = 0.9;
    double alpha = 0.0001;
    double gamma = 1e-4;
    double gdiff = 0.000001;

    bool do_BB1e2 = true;
    bool do_BB1 = false;
    bool do_BB2 = false;
    double neg_BB1_fallback = 0.11;
    double neg_BB2_fallback = 0.12;

    m_iterations = 0;
    n_armijo_backtracks = 0;

    int nc = sysd.CountActiveConstraints();
    if (verbose)
        std::cout << "\n-----Barzilai-Borwein, solving nc=" << nc << "unknowns" << std::endl;

    // Allocate auxiliary vectors
    ChVectorDynamic<> ml(nc);
    ChVectorDynamic<> ml_candidate(nc);
    ChVectorDynamic<> mg(nc);
    ChVectorDynamic<> mg_p(nc);
    ChVectorDynamic<> ml_p(nc);
    ChVectorDynamic<> mdir(nc);
    ChVectorDynamic<> mb(nc);
    ChVectorDynamic<> mb_tmp(nc);
    ChVectorDynamic<> ms(nc);
    ChVectorDynamic<> my(nc);
    ChVectorDynamic<> mDg(nc);

    // Update auxiliary data in constraints
    // Average entries for friction constraints
    sysd.SchurComplementUpdateConstraints(true);

    // Calculate the Schur complement transformed RHS
    // Cache M^-1 * f in 'Mif'
    ChVectorDynamic<> Mif;
    sysd.SchurComplementRHS(mb, &Mif);

    // Initialize lambdas
    if (m_warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.setZero();

    // Initial projection of ml   ***TO DO***?
    sysd.ConstraintsProject(ml);

    // Fallback solution
    lastgoodres = 1e30;
    ml_candidate = ml;

    // Calculate g = grad(0.5*l'*N*l-l'*b) = N*l-b
    // If using a diagonal preconditioner, cache the inverse diagonal of N
    ChVectorDynamic<> iD(nc);

    // 1) g = N*l 
    if (m_use_precond)
        sysd.SchurComplementProduct(mg, ml, &iD);
    else
        sysd.SchurComplementProduct(mg, ml);

    // 2) g = N*l -b
    mg -= mb;

    mg_p = mg;

    // initial norm of the gradient
    double mg_p_init_norm = std::max(1e-10, mg_p.norm());

    // Iterations

    double mf_p = 0;

    // Objective at the current iterate:  f(l) = 0.5*l'*N*l - l'*b.
    // Using g = N*l - b this is  f = 0.5*l'*(g + b) - l'*b = 0.5*l'*(g - b).
    double mf = 0.5 * ml.dot(mg - mb);

    // Objective value at each *accepted* iterate, used by the non-monotone line search.
    std::vector<double> f_hist;
    f_hist.push_back(mf);

    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // Dg = Di*g;
        mDg = mg;
        if (m_use_precond)
            mDg = mDg.array() * iD.array();

        // dir  = [P(l - alpha*Dg) - l]
        mdir = ml - alpha * mDg;        // dir = l - alpha*Dg
        sysd.ConstraintsProject(mdir);  // dir = P(l - alpha*Dg)
        mdir -= ml;                     // dir = P(l - alpha*Dg) - l

        // dTg = dir'*g;
        double dTg = mdir.dot(mg);

        // BB dir backward!? fallback to nonpreconditioned dir
        if (dTg > 1e-8) {
            // dir  = [P(l - alpha*g) - l]
            mdir = ml - alpha * mg;         // dir = l - alpha*g
            sysd.ConstraintsProject(mdir);  // dir = P(l - alpha*g) ...
            mdir -= ml;                     // dir = P(l - alpha*g) - l
            // dTg = d'*g;
            dTg = mdir.dot(mg);
        }

        double lambda = 1;

        int n_backtracks = 0;
        bool armijo_repeat = true;

        while (armijo_repeat) {
            // l_p = l + lambda*dir;
            ml_p = ml + lambda * mdir;

            // m_tmp = Nl_p = N*l_p;
            sysd.SchurComplementProduct(mb_tmp, ml_p);

            // g_p = N * l_p - b  = Nl_p - b
            mg_p = mb_tmp - mb;

            // f_p = 0.5*l_p'*N*l_p - l_p'*b  = l_p'*(0.5*Nl_p - b);
            mf_p = ml_p.dot(0.5 * mb_tmp - mb);

            // Non-monotone (Grippo-Lampariello-Lucidi) Armijo condition: accept the
            // trial step if
            //     f(l_p) <= max_{0 <= h <= n_armijo} f(l_{k-h}) + gamma*lambda*dir'*g
            // where the maximum is taken over the objective at the most recent accepted
            // iterates, including the current one.
            //
            // NOTE: this reference value was previously seeded with +10e29 and the loop
            // could only raise it, so the acceptance test was always true and the line
            // search never ran.  The history was also appended once per *trial* rather
            // than once per accepted iterate, which corrupted its indexing as soon as a
            // backtrack did occur.  Both are fixed here.
            double f_ref = -std::numeric_limits<double>::max();
            int n_hist = static_cast<int>(f_hist.size());
            int n_back = std::min(n_hist, this->n_armijo + 1);
            for (int h = 0; h < n_back; h++)
                f_ref = std::max(f_ref, f_hist[n_hist - 1 - h]);

            if (mf_p > f_ref + gamma * lambda * dTg && n_backtracks < this->max_armijo_backtrace) {
                armijo_repeat = true;
                // Safeguarded quadratic interpolation of the step length.
                double denom = 2 * (mf_p - mf - lambda * dTg);
                double lambdanew = (denom != 0) ? -lambda * lambda * dTg / denom : sigma_min * lambda;
                lambda = std::max(sigma_min * lambda, std::min(sigma_max * lambda, lambdanew));
                n_backtracks++;
                n_armijo_backtracks++;
                if (verbose)
                    std::cout << " Repeat Armijo, new lambda=" << lambda << std::endl;
            } else {
                armijo_repeat = false;
            }
        }

        // Record the objective at the accepted iterate.  Only the last n_armijo+1
        // entries are ever read.
        f_hist.push_back(mf_p);
        if (static_cast<int>(f_hist.size()) > this->n_armijo + 1)
            f_hist.erase(f_hist.begin());
        mf = mf_p;

        ms = ml_p - ml;  // s = l_p - l;
        my = mg_p - mg;  // y = g_p - g;
        ml = ml_p;       // l = l_p;
        mg = mg_p;       // g = g_p;

        if (((do_BB1e2) && (iter % 2 == 0)) || do_BB1) {
            if (m_use_precond)
                mb_tmp = ms.array() / iD.array();
            else
                mb_tmp = ms;
            double sDs = ms.dot(mb_tmp);
            double sy = ms.dot(my);
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                double alph = sDs / sy;  // (s,Ds)/(s,y)   BB1
                alpha = std::min(a_max, std::max(a_min, alph));
            }
        }

        /*
        // this is a modified rayleight quotient - looks like it works anyway...
        if (((do_BB1e2) && (iter%2 ==0)) || do_BB1)
        {
            double ss = ms.MatrDot(ms,ms);
            mb_tmp = my;
            if (m_use_precond)
                mb_tmp.MatrDivScale(mD);
            double sDy = ms.MatrDot(ms, mb_tmp);
            if (sDy <= 0)
            {
                alpha = neg_BB1_fallback;
            }
            else
            {
                double alph = ss / sDy;  // (s,s)/(s,Di*y)   BB1 (modified version)
                alpha = std::min (a_max, std::max(a_min, alph));
            }
        }
        */

        if (((do_BB1e2) && (iter % 2 != 0)) || do_BB2) {
            double sy = ms.dot(my);
            if (m_use_precond)
                mb_tmp = my.array() * iD.array();
            else
                mb_tmp = my;
            double yDy = my.dot(mb_tmp);
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                double alph = sy / yDy;  // (s,y)/(y,Di*y)   BB2
                alpha = std::min(a_max, std::max(a_min, alph));
            }
        }

        // Project the gradient (for rollback strategy)
        // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
        mb_tmp = ml - gdiff * mg;
        sysd.ConstraintsProject(mb_tmp);     // mb_tmp = ProjectionOperator(l - gdiff * g)
        mb_tmp = (ml - mb_tmp) / gdiff;      // mb_tmp = [l - ProjectionOperator(l - gdiff * g)] / gdiff
        double g_proj_norm = mb_tmp.norm();  // infinity norm is faster..

        // Rollback solution: the last best candidate ('l' with lowest projected gradient)
        // in fact the method is not monotone and it is quite 'noisy', if we do not
        // do this, a prematurely truncated iteration might give a crazy result.
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
        }

        // METRICS - convergence, plots, etc

        double maxdeltalambda = ms.lpNorm<Eigen::Infinity>();
        double maxd = lastgoodres / mg_p_init_norm;

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxd, maxdeltalambda, iter);

        if (verbose)
            std::cout << "  iter=" << iter << "   f=" << mf_p << "  |d|=" << maxd << "  |s|=" << maxdeltalambda
                      << std::endl;

        m_iterations++;

        if (maxd < m_tolerance) {
            if (verbose)
                std::cout << "Converged at iter: " << iter << " with residual: " << maxd << std::endl;
            break;
        }
    }

    // Fallback to best found solution (might be useful because of nonmonotonicity)
    ml = ml_candidate;

    // Resulting DUAL variables:
    // store ml temporary vector into ChConstraint 'l_i' multipliers
    sysd.FromVectorToConstraints(ml);

    // Resulting PRIMAL variables:
    // compute the primal variables as   v = (M^-1)(f + Cq'*l)
    sysd.SchurComplementIncrementVariables(&Mif);

    if (verbose)
        std::cout << "-----" << std::endl;

    return lastgoodres;
}

// -----------------------------------------------------------------------------

void ChSolverBB::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolverBB>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(n_armijo);
    archive_out << CHNVP(max_armijo_backtrace);
    archive_out << CHNVP(m_use_precond);
}

void ChSolverBB::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolverBB>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(n_armijo);
    archive_in >> CHNVP(max_armijo_backtrace);
    archive_in >> CHNVP(m_use_precond);
}

}  // end namespace chrono
