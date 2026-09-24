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
// Unit test for the solver constraint kernels ComputeJacobianTimesState and
// IncrementState of the two-body, three-body-shaft and tuple constraints.
// The results are compared against a reference implementation that operates
// on the runtime-size state vector returned by ChVariables::State().
//
// The kernels use fixed-size Eigen expressions and the reference uses
// runtime-size ones. Eigen does not guarantee that the two evaluation paths
// round identically (e.g. different reduction order or FMA contraction), so the
// pass criterion is 32 machine epsilons times the sum of the magnitudes of the
// terms involved. This still catches any indexing, sign,
// size or missing-term error. The number of results that are not bitwise
// identical is printed for information; it is 0 on the platforms checked so
// far (GCC and Clang, x86_64 SSE2/AVX2+FMA and arm64).
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/solver/ChConstraintThreeBBShaft.h"
#include "chrono/solver/ChConstraintTuple.h"
#include "chrono/solver/ChConstraintTwoBodies.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesShaft.h"

using namespace chrono;

namespace {

std::mt19937 rng(42);

// Random value spanning several orders of magnitude, so that rounding in the dot products matters.
double Rand() {
    std::uniform_real_distribution<double> val(-1.0, 1.0);
    std::uniform_int_distribution<int> expo(-6, 6);
    return std::ldexp(val(rng), expo(rng));
}

void Randomize(ChVectorRef v) {
    for (Eigen::Index i = 0; i < v.size(); i++)
        v(i) = Rand();
}

void Randomize(ChRowVectorRef v) {
    for (Eigen::Index i = 0; i < v.size(); i++)
        v(i) = Rand();
}

// Reference kernels, written on the runtime-size state vector as in the original implementation.
template <int N>
double RefDot(const ChRowVectorN<double, N>& cq, ChVariables& var) {
    return cq * var.State();
}

// Sum of the magnitudes of the terms of the dot product, used to scale the tolerance.
template <int N>
double AbsDot(const ChRowVectorN<double, N>& cq, ChVariables& var) {
    return cq.cwiseAbs() * var.State().cwiseAbs();
}

// Magnitudes of the terms of the increment, used to scale the tolerance.
template <int N>
ChVectorDynamic<double> AbsIncrement(const ChVectorN<double, N>& eq, ChVariables& var, double deltal) {
    return var.State().cwiseAbs() + (eq * deltal).cwiseAbs();
}

template <int N>
void RefIncrement(const ChVectorN<double, N>& eq, ChVariables& var, double deltal) {
    var.State() += eq * deltal;
}

// Compares kernel results with the reference within a tolerance scaled by the magnitude of the terms,
// and counts the results that are not bitwise identical.
class Checker {
  public:
    explicit Checker(const std::string& name) : m_name(name) {}
    ~Checker() {
        std::cout << "[ bitwise  ] " << m_name << ": " << m_mismatch << " of " << m_count
                  << " results differ from the reference" << std::endl;
    }

    void Value(double got, double ref, double scale) {
        m_count++;
        if (got != ref)
            m_mismatch++;
        EXPECT_LE(std::abs(got - ref), kTol * scale) << m_name << ": got " << got << ", expected " << ref;
    }

    void State(ChVectorConstRef got, ChVectorConstRef ref, ChVectorConstRef scale) {
        ASSERT_EQ(got.size(), ref.size());
        for (Eigen::Index i = 0; i < got.size(); i++)
            Value(got(i), ref(i), scale(i));
    }

  private:
    static constexpr double kTol = 32 * std::numeric_limits<double>::epsilon();
    std::string m_name;
    int m_count = 0;
    int m_mismatch = 0;
};

}  // namespace

// -----------------------------------------------------------------------------

TEST(ChConstraintState, TwoBodies) {
    ChVariablesBodyOwnMass va, vb;
    ChConstraintTwoBodies c(&va, &vb);
    Checker check("TwoBodies");

    for (int trial = 0; trial < 1000; trial++) {
        // Exercise the inactive-variable branches too
        va.SetDisabled(trial % 7 == 3);
        vb.SetDisabled(trial % 5 == 2);

        Randomize(c.Get_Cq_a());
        Randomize(c.Get_Cq_b());
        Randomize(c.Get_Eq_a());
        Randomize(c.Get_Eq_b());
        Randomize(va.State());
        Randomize(vb.State());
        double deltal = Rand();

        ChRowVectorN<double, 6> cq_a = c.Get_Cq_a();
        ChRowVectorN<double, 6> cq_b = c.Get_Cq_b();
        ChVectorN<double, 6> eq_a = c.Get_Eq_a();
        ChVectorN<double, 6> eq_b = c.Get_Eq_b();

        double ref = 0;
        double scale = 0;
        if (va.IsActive()) {
            ref += RefDot<6>(cq_a, va);
            scale += AbsDot<6>(cq_a, va);
        }
        if (vb.IsActive()) {
            ref += RefDot<6>(cq_b, vb);
            scale += AbsDot<6>(cq_b, vb);
        }
        check.Value(c.ComputeJacobianTimesState(), ref, scale);

        ChVariablesBodyOwnMass ra, rb;
        ra.State() = va.State();
        rb.State() = vb.State();
        ChVectorDynamic<double> sa = AbsIncrement<6>(eq_a, ra, deltal);
        ChVectorDynamic<double> sb = AbsIncrement<6>(eq_b, rb, deltal);
        if (va.IsActive())
            RefIncrement<6>(eq_a, ra, deltal);
        if (vb.IsActive())
            RefIncrement<6>(eq_b, rb, deltal);
        c.IncrementState(deltal);
        check.State(va.State(), ra.State(), sa);
        check.State(vb.State(), rb.State(), sb);
    }
}

TEST(ChConstraintState, ThreeBBShaft) {
    ChVariablesBodyOwnMass va, vb;
    ChVariablesShaft vc;
    ChConstraintThreeBBShaft c(&va, &vb, &vc);
    Checker check("ThreeBBShaft");

    for (int trial = 0; trial < 1000; trial++) {
        va.SetDisabled(trial % 7 == 3);
        vb.SetDisabled(trial % 5 == 2);

        Randomize(c.Get_Cq_a());
        Randomize(c.Get_Cq_b());
        Randomize(c.Get_Cq_c());
        Randomize(c.Get_Eq_a());
        Randomize(c.Get_Eq_b());
        Randomize(c.Get_Eq_c());
        Randomize(va.State());
        Randomize(vb.State());
        Randomize(vc.State());
        double deltal = Rand();

        ChRowVectorN<double, 6> cq_a = c.Get_Cq_a();
        ChRowVectorN<double, 6> cq_b = c.Get_Cq_b();
        ChVectorN<double, 6> eq_a = c.Get_Eq_a();
        ChVectorN<double, 6> eq_b = c.Get_Eq_b();

        double ref = 0;
        double scale = 0;
        if (va.IsActive()) {
            ref += RefDot<6>(cq_a, va);
            scale += AbsDot<6>(cq_a, va);
        }
        if (vb.IsActive()) {
            ref += RefDot<6>(cq_b, vb);
            scale += AbsDot<6>(cq_b, vb);
        }
        ref += c.Get_Cq_c()(0) * vc.State()(0);
        scale += std::abs(c.Get_Cq_c()(0) * vc.State()(0));
        check.Value(c.ComputeJacobianTimesState(), ref, scale);

        ChVariablesBodyOwnMass ra, rb;
        ra.State() = va.State();
        rb.State() = vb.State();
        ChVectorDynamic<double> sa = AbsIncrement<6>(eq_a, ra, deltal);
        ChVectorDynamic<double> sb = AbsIncrement<6>(eq_b, rb, deltal);
        double qc = vc.State()(0) + c.Get_Eq_c()(0) * deltal;
        double sc = std::abs(vc.State()(0)) + std::abs(c.Get_Eq_c()(0) * deltal);
        if (va.IsActive())
            RefIncrement<6>(eq_a, ra, deltal);
        if (vb.IsActive())
            RefIncrement<6>(eq_b, rb, deltal);
        c.IncrementState(deltal);
        check.State(va.State(), ra.State(), sa);
        check.State(vb.State(), rb.State(), sb);
        check.Value(vc.State()(0), qc, sc);
    }
}

// Tuple with one variable set, e.g. a single FEA node (3 DOF) or rigid body (6 DOF).
template <int N>
void CheckTuple1() {
    ChVariablesGeneric v(N);
    ChConstraintTuple_1vars<N> t(&v);
    Checker check("Tuple1<" + std::to_string(N) + ">");

    for (int trial = 0; trial < 1000; trial++) {
        v.SetDisabled(trial % 7 == 3);
        Randomize(t.Cq1());
        Randomize(t.Eq1());
        Randomize(v.State());
        double deltal = Rand();

        ChRowVectorN<double, N> cq = t.Cq1();
        ChVectorN<double, N> eq = t.Eq1();

        double ref = v.IsActive() ? RefDot<N>(cq, v) : 0.0;
        double scale = v.IsActive() ? AbsDot<N>(cq, v) : 0.0;
        check.Value(t.ComputeJacobianTimesState(), ref, scale);

        ChVariablesGeneric r(N);
        r.State() = v.State();
        ChVectorDynamic<double> sr = AbsIncrement<N>(eq, r, deltal);
        if (v.IsActive())
            RefIncrement<N>(eq, r, deltal);
        t.IncrementState(deltal);
        check.State(v.State(), r.State(), sr);
    }
}

TEST(ChConstraintState, Tuple1) {
    CheckTuple1<3>();
    CheckTuple1<6>();
}

template <int N1, int N2>
void CheckTuple2() {
    ChVariablesGeneric v1(N1), v2(N2);
    ChConstraintTuple_2vars<N1, N2> t(&v1, &v2);
    Checker check("Tuple2<" + std::to_string(N1) + "," + std::to_string(N2) + ">");

    for (int trial = 0; trial < 1000; trial++) {
        v1.SetDisabled(trial % 7 == 3);
        v2.SetDisabled(trial % 5 == 2);
        Randomize(t.Cq1());
        Randomize(t.Cq2());
        Randomize(t.Eq1());
        Randomize(t.Eq2());
        Randomize(v1.State());
        Randomize(v2.State());
        double deltal = Rand();

        ChRowVectorN<double, N1> cq1 = t.Cq1();
        ChRowVectorN<double, N2> cq2 = t.Cq2();
        ChVectorN<double, N1> eq1 = t.Eq1();
        ChVectorN<double, N2> eq2 = t.Eq2();

        double ref = 0;
        double scale = 0;
        if (v1.IsActive()) {
            ref += RefDot<N1>(cq1, v1);
            scale += AbsDot<N1>(cq1, v1);
        }
        if (v2.IsActive()) {
            ref += RefDot<N2>(cq2, v2);
            scale += AbsDot<N2>(cq2, v2);
        }
        check.Value(t.ComputeJacobianTimesState(), ref, scale);

        ChVariablesGeneric r1(N1), r2(N2);
        r1.State() = v1.State();
        r2.State() = v2.State();
        ChVectorDynamic<double> s1 = AbsIncrement<N1>(eq1, r1, deltal);
        ChVectorDynamic<double> s2 = AbsIncrement<N2>(eq2, r2, deltal);
        if (v1.IsActive())
            RefIncrement<N1>(eq1, r1, deltal);
        if (v2.IsActive())
            RefIncrement<N2>(eq2, r2, deltal);
        t.IncrementState(deltal);
        check.State(v1.State(), r1.State(), s1);
        check.State(v2.State(), r2.State(), s2);
    }
}

TEST(ChConstraintState, Tuple2) {
    CheckTuple2<6, 6>();
    CheckTuple2<3, 3>();
    CheckTuple2<6, 3>();
}

template <int N1, int N2, int N3>
void CheckTuple3() {
    ChVariablesGeneric v1(N1), v2(N2), v3(N3);
    ChConstraintTuple_3vars<N1, N2, N3> t(&v1, &v2, &v3);
    Checker check("Tuple3<" + std::to_string(N1) + "," + std::to_string(N2) + "," + std::to_string(N3) + ">");

    for (int trial = 0; trial < 1000; trial++) {
        v1.SetDisabled(trial % 7 == 3);
        v2.SetDisabled(trial % 5 == 2);
        v3.SetDisabled(trial % 11 == 4);
        Randomize(t.Cq1());
        Randomize(t.Cq2());
        Randomize(t.Cq3());
        Randomize(t.Eq1());
        Randomize(t.Eq2());
        Randomize(t.Eq3());
        Randomize(v1.State());
        Randomize(v2.State());
        Randomize(v3.State());
        double deltal = Rand();

        ChRowVectorN<double, N1> cq1 = t.Cq1();
        ChRowVectorN<double, N2> cq2 = t.Cq2();
        ChRowVectorN<double, N3> cq3 = t.Cq3();
        ChVectorN<double, N1> eq1 = t.Eq1();
        ChVectorN<double, N2> eq2 = t.Eq2();
        ChVectorN<double, N3> eq3 = t.Eq3();

        double ref = 0;
        double scale = 0;
        if (v1.IsActive()) {
            ref += RefDot<N1>(cq1, v1);
            scale += AbsDot<N1>(cq1, v1);
        }
        if (v2.IsActive()) {
            ref += RefDot<N2>(cq2, v2);
            scale += AbsDot<N2>(cq2, v2);
        }
        if (v3.IsActive()) {
            ref += RefDot<N3>(cq3, v3);
            scale += AbsDot<N3>(cq3, v3);
        }
        check.Value(t.ComputeJacobianTimesState(), ref, scale);

        ChVariablesGeneric r1(N1), r2(N2), r3(N3);
        r1.State() = v1.State();
        r2.State() = v2.State();
        r3.State() = v3.State();
        ChVectorDynamic<double> s1 = AbsIncrement<N1>(eq1, r1, deltal);
        ChVectorDynamic<double> s2 = AbsIncrement<N2>(eq2, r2, deltal);
        ChVectorDynamic<double> s3 = AbsIncrement<N3>(eq3, r3, deltal);
        if (v1.IsActive())
            RefIncrement<N1>(eq1, r1, deltal);
        if (v2.IsActive())
            RefIncrement<N2>(eq2, r2, deltal);
        if (v3.IsActive())
            RefIncrement<N3>(eq3, r3, deltal);
        t.IncrementState(deltal);
        check.State(v1.State(), r1.State(), s1);
        check.State(v2.State(), r2.State(), s2);
        check.State(v3.State(), r3.State(), s3);
    }
}

TEST(ChConstraintState, Tuple3) {
    CheckTuple3<3, 3, 3>();
    CheckTuple3<6, 6, 6>();
}
