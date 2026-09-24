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
// Test of ChSystemDescriptor::ConstraintsProject on a mix of bilateral (LOCK),
// unilateral, boxed, frictional contact and rolling contact constraints.
// The result must be bitwise identical to a reference projection that scatters
// the multipliers to all active constraints, calls Project() on each of them in
// order, and gathers the multipliers back.
//
// =============================================================================

#include <memory>
#include <random>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"
#include "chrono/solver/ChConstraintContactNormal.h"
#include "chrono/solver/ChConstraintContactTangential.h"
#include "chrono/solver/ChConstraintRollingNormal.h"
#include "chrono/solver/ChConstraintRollingTangential.h"

using namespace chrono;

class ConstraintMix {
  public:
    // Create 'groups' groups of constraints of all supported kinds, interleaved.
    ConstraintMix(int groups, std::mt19937& rng) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int g = 0; g < groups; g++) {
            // bilateral
            auto lock = Add(new ChConstraintTwoGeneric());
            (void)lock;

            // unilateral
            auto uni = Add(new ChConstraintTwoGeneric());
            uni->SetMode(ChConstraint::Mode::UNILATERAL);

            // boxed (LOCK mode, but with a non-identity projection)
            auto boxed = Add(new ChConstraintTwoGenericBoxed());
            double lo = -u01(rng);
            boxed->SetBoxedMinMax(lo, lo + u01(rng));

            // frictional contact (one in four frictionless)
            auto cn = Add(new ChConstraintContactNormal());
            auto cu = Add(new ChConstraintContactTangential());
            auto cv = Add(new ChConstraintContactTangential());
            cn->SetTangentialConstraintU(cu);
            cn->SetTangentialConstraintV(cv);
            cn->SetFrictionCoefficient(g % 4 == 0 ? 0.0 : 0.1 + u01(rng));
            cn->SetCohesion(g % 3 == 0 ? 0.1 * u01(rng) : 0.0);

            // bilateral between contact and rolling constraints
            Add(new ChConstraintTwoGeneric());

            // rolling contact, coupled to the frictional contact above
            auto rn = Add(new ChConstraintRollingNormal());
            auto ru = Add(new ChConstraintRollingTangential());
            auto rv = Add(new ChConstraintRollingTangential());
            rn->SetNormalConstraint(cn);
            rn->SetRollingConstraintU(ru);
            rn->SetRollingConstraintV(rv);
            rn->SetRollingFrictionCoefficient(g % 2 == 0 ? 0.0f : 0.05f + 0.1f * (float)u01(rng));
            rn->SetSpinningFrictionCoefficient(g % 5 == 0 ? 0.0f : 0.05f + 0.1f * (float)u01(rng));

            // inactive constraints (disabled bilateral and free mode)
            auto dis = Add(new ChConstraintTwoGeneric());
            dis->SetDisabled(true);
            auto fr = Add(new ChConstraintTwoGeneric());
            fr->SetMode(ChConstraint::Mode::FREE);
        }
    }

    void Insert(ChSystemDescriptor& sysd, int stride = 1) {
        sysd.BeginInsertion();
        for (size_t i = 0; i < constraints.size(); i += stride)
            sysd.InsertConstraint(constraints[i].get());
        sysd.EndInsertion();
    }

    std::vector<std::unique_ptr<ChConstraint>> constraints;

  private:
    template <class T>
    T* Add(T* c) {
        c->SetValid(true);
        constraints.push_back(std::unique_ptr<ChConstraint>(c));
        return c;
    }
};

// Reference projection: scatter to all active constraints, project all, gather back.
static void ReferenceProject(ChSystemDescriptor& sysd, ChVectorDynamic<>& l) {
    for (auto c : sysd.GetConstraints())
        if (c->IsActive())
            c->SetLagrangeMultiplier(l(c->GetOffset()));
    for (auto c : sysd.GetConstraints())
        if (c->IsActive())
            c->Project();
    for (auto c : sysd.GetConstraints())
        if (c->IsActive())
            l(c->GetOffset()) = c->GetLagrangeMultiplier();
}

// Constraints left out of the descriptor can still be read and written by the cone projections of their partners.
// Save and restore all multipliers so that both projections start from the same state.
static std::vector<double> SaveMultipliers(const ConstraintMix& mix) {
    std::vector<double> l;
    for (const auto& c : mix.constraints)
        l.push_back(c->GetLagrangeMultiplier());
    return l;
}

static void RestoreMultipliers(ConstraintMix& mix, const std::vector<double>& l) {
    for (size_t i = 0; i < l.size(); i++)
        mix.constraints[i]->SetLagrangeMultiplier(l[i]);
}

static void CheckProjection(ChSystemDescriptor& sysd, ConstraintMix& mix, std::mt19937& rng, int trials) {
    std::normal_distribution<double> nd(0.0, 1.0);
    unsigned int n = sysd.CountActiveConstraints();
    ASSERT_GT(n, 0u);

    for (int t = 0; t < trials; t++) {
        ChVectorDynamic<> l(n);
        for (unsigned int i = 0; i < n; i++)
            l(i) = nd(rng);
        // a few exact zeros and tiny values exercise the special branches of the cone projections
        l(t % n) = 0.0;
        l((3 * t + 1) % n) = 1e-15;

        ChVectorDynamic<> l_test = l;
        ChVectorDynamic<> l_ref = l;

        auto saved = SaveMultipliers(mix);
        sysd.ConstraintsProject(l_test);

        // multipliers stored in the projected constraints must match the returned vector
        for (auto c : sysd.GetConstraints()) {
            if (c->IsActive() && c->GetMode() != ChConstraint::Mode::LOCK)
                ASSERT_EQ(c->GetLagrangeMultiplier(), l_test(c->GetOffset()));
        }

        auto after_test = SaveMultipliers(mix);
        RestoreMultipliers(mix, saved);
        ReferenceProject(sysd, l_ref);

        ASSERT_EQ(l_test.size(), l_ref.size());
        for (unsigned int i = 0; i < n; i++)
            ASSERT_EQ(l_test(i), l_ref(i)) << "entry " << i << " trial " << t;

        // a second projection of the projected vector must also match
        RestoreMultipliers(mix, after_test);
        saved = SaveMultipliers(mix);
        ChVectorDynamic<> l_again = l_test;
        sysd.ConstraintsProject(l_again);
        RestoreMultipliers(mix, saved);
        ChVectorDynamic<> l_again_ref = l_test;
        ReferenceProject(sysd, l_again_ref);
        for (unsigned int i = 0; i < n; i++)
            ASSERT_EQ(l_again(i), l_again_ref(i)) << "entry " << i << " trial " << t << " (second pass)";
    }
}

TEST(ChSystemDescriptorTest, constraints_project_mixed) {
    std::mt19937 rng(42);
    ConstraintMix mix(40, rng);
    ChSystemDescriptor sysd;
    mix.Insert(sysd);
    CheckProjection(sysd, mix, rng, 50);
}

TEST(ChSystemDescriptorTest, constraints_project_reinsert) {
    // Re-assembling the descriptor with a different set of constraints must update the projected set.
    std::mt19937 rng(7);
    ConstraintMix mix(25, rng);
    ChSystemDescriptor sysd;

    mix.Insert(sysd);
    CheckProjection(sysd, mix, rng, 10);

    // Drop every other constraint (breaks some contact triplets on purpose: the pointers stay valid)
    mix.Insert(sysd, 2);
    CheckProjection(sysd, mix, rng, 10);

    // Change modes and active flags between assemblies
    for (size_t i = 0; i < mix.constraints.size(); i += 3) {
        auto c = mix.constraints[i].get();
        if (c->GetMode() == ChConstraint::Mode::LOCK && dynamic_cast<ChConstraintTwoGenericBoxed*>(c) == nullptr)
            c->SetMode(ChConstraint::Mode::UNILATERAL);
        else if (c->GetMode() == ChConstraint::Mode::UNILATERAL)
            c->SetMode(ChConstraint::Mode::LOCK);
    }
    for (size_t i = 1; i < mix.constraints.size(); i += 7)
        mix.constraints[i]->SetDisabled(!mix.constraints[i]->IsDisabled());
    mix.Insert(sysd);
    CheckProjection(sysd, mix, rng, 10);
}

TEST(ChSystemDescriptorTest, constraints_project_bilateral_only) {
    // With only bilateral constraints the projection is the identity.
    std::vector<std::unique_ptr<ChConstraintTwoGeneric>> cs;
    ChSystemDescriptor sysd;
    sysd.BeginInsertion();
    for (int i = 0; i < 20; i++) {
        cs.emplace_back(new ChConstraintTwoGeneric());
        cs.back()->SetValid(true);
        sysd.InsertConstraint(cs.back().get());
    }
    sysd.EndInsertion();

    ChVectorDynamic<> l(20);
    for (int i = 0; i < 20; i++)
        l(i) = -1.0 + 0.1 * i;
    ChVectorDynamic<> l0 = l;
    sysd.ConstraintsProject(l);
    for (int i = 0; i < 20; i++)
        ASSERT_EQ(l(i), l0(i));
}
