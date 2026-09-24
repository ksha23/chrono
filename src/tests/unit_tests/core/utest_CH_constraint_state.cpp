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
// The results are compared bitwise against a reference implementation that
// operates on the runtime-size state vector returned by ChVariables::State().
//
// =============================================================================

#include <random>
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

template <int N>
void RefIncrement(const ChVectorN<double, N>& eq, ChVariables& var, double deltal) {
    var.State() += eq * deltal;
}

// Bitwise comparison of two state vectors.
void ExpectSameState(ChVectorConstRef a, ChVectorConstRef b) {
    ASSERT_EQ(a.size(), b.size());
    for (Eigen::Index i = 0; i < a.size(); i++)
        EXPECT_EQ(a(i), b(i)) << "entry " << i;
}

}  // namespace

// -----------------------------------------------------------------------------

TEST(ChConstraintState, TwoBodies) {
    ChVariablesBodyOwnMass va, vb;
    ChConstraintTwoBodies c(&va, &vb);

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
        if (va.IsActive())
            ref += RefDot<6>(cq_a, va);
        if (vb.IsActive())
            ref += RefDot<6>(cq_b, vb);
        EXPECT_EQ(c.ComputeJacobianTimesState(), ref);

        ChVariablesBodyOwnMass ra, rb;
        ra.State() = va.State();
        rb.State() = vb.State();
        if (va.IsActive())
            RefIncrement<6>(eq_a, ra, deltal);
        if (vb.IsActive())
            RefIncrement<6>(eq_b, rb, deltal);
        c.IncrementState(deltal);
        ExpectSameState(va.State(), ra.State());
        ExpectSameState(vb.State(), rb.State());
    }
}

TEST(ChConstraintState, ThreeBBShaft) {
    ChVariablesBodyOwnMass va, vb;
    ChVariablesShaft vc;
    ChConstraintThreeBBShaft c(&va, &vb, &vc);

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
        if (va.IsActive())
            ref += RefDot<6>(cq_a, va);
        if (vb.IsActive())
            ref += RefDot<6>(cq_b, vb);
        ref += c.Get_Cq_c()(0) * vc.State()(0);
        EXPECT_EQ(c.ComputeJacobianTimesState(), ref);

        ChVariablesBodyOwnMass ra, rb;
        ra.State() = va.State();
        rb.State() = vb.State();
        double qc = vc.State()(0) + c.Get_Eq_c()(0) * deltal;
        if (va.IsActive())
            RefIncrement<6>(eq_a, ra, deltal);
        if (vb.IsActive())
            RefIncrement<6>(eq_b, rb, deltal);
        c.IncrementState(deltal);
        ExpectSameState(va.State(), ra.State());
        ExpectSameState(vb.State(), rb.State());
        EXPECT_EQ(vc.State()(0), qc);
    }
}

// Tuple with one variable set, e.g. a single FEA node (3 DOF) or rigid body (6 DOF).
template <int N>
void CheckTuple1() {
    ChVariablesGeneric v(N);
    ChConstraintTuple_1vars<N> t(&v);

    for (int trial = 0; trial < 1000; trial++) {
        v.SetDisabled(trial % 7 == 3);
        Randomize(t.Cq1());
        Randomize(t.Eq1());
        Randomize(v.State());
        double deltal = Rand();

        ChRowVectorN<double, N> cq = t.Cq1();
        ChVectorN<double, N> eq = t.Eq1();

        double ref = v.IsActive() ? RefDot<N>(cq, v) : 0.0;
        EXPECT_EQ(t.ComputeJacobianTimesState(), ref);

        ChVariablesGeneric r(N);
        r.State() = v.State();
        if (v.IsActive())
            RefIncrement<N>(eq, r, deltal);
        t.IncrementState(deltal);
        ExpectSameState(v.State(), r.State());
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
        if (v1.IsActive())
            ref += RefDot<N1>(cq1, v1);
        if (v2.IsActive())
            ref += RefDot<N2>(cq2, v2);
        EXPECT_EQ(t.ComputeJacobianTimesState(), ref);

        ChVariablesGeneric r1(N1), r2(N2);
        r1.State() = v1.State();
        r2.State() = v2.State();
        if (v1.IsActive())
            RefIncrement<N1>(eq1, r1, deltal);
        if (v2.IsActive())
            RefIncrement<N2>(eq2, r2, deltal);
        t.IncrementState(deltal);
        ExpectSameState(v1.State(), r1.State());
        ExpectSameState(v2.State(), r2.State());
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
        if (v1.IsActive())
            ref += RefDot<N1>(cq1, v1);
        if (v2.IsActive())
            ref += RefDot<N2>(cq2, v2);
        if (v3.IsActive())
            ref += RefDot<N3>(cq3, v3);
        EXPECT_EQ(t.ComputeJacobianTimesState(), ref);

        ChVariablesGeneric r1(N1), r2(N2), r3(N3);
        r1.State() = v1.State();
        r2.State() = v2.State();
        r3.State() = v3.State();
        if (v1.IsActive())
            RefIncrement<N1>(eq1, r1, deltal);
        if (v2.IsActive())
            RefIncrement<N2>(eq2, r2, deltal);
        if (v3.IsActive())
            RefIncrement<N3>(eq3, r3, deltal);
        t.IncrementState(deltal);
        ExpectSameState(v1.State(), r1.State());
        ExpectSameState(v2.State(), r2.State());
        ExpectSameState(v3.State(), r3.State());
    }
}

TEST(ChConstraintState, Tuple3) {
    CheckTuple3<3, 3, 3>();
    CheckTuple3<6, 6, 6>();
}
