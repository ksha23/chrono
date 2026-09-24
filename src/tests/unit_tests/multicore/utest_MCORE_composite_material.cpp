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
// Unit test for the per-contact composite material data loaded by the
// Chrono::Multicore contact containers (NSC and SMC).
// Several bodies with distinct materials rest on a ground box and on each other.
// After one step, the test checks that the material data stored for each contact
// equals the composite material of the two bodies in contact, both without and
// with a user-provided AddContactCallback (which must fire for every contact,
// receive the correct shapes, and be able to override the composite material).
//
// =============================================================================

#include <atomic>

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "../ut_utils.h"

using namespace chrono;

// Material of the first collision shape of a body.
static std::shared_ptr<ChContactMaterial> BodyMaterial(const ChBody& body) {
    return body.GetCollisionModel()->GetShapeInstance(0).shape->GetMaterial();
}

// Callback that overrides the composite friction with the sum of the two shape frictions and checks that the shapes
// passed in the collision info belong to the collision models passed in the collision info.
class TestAddContactCallback : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const ChCollisionInfo& cinfo, ChContactMaterialComposite* mat) override {
        num_calls++;
        bool okA = cinfo.shapeA->GetMaterial() == BodyMaterial(*static_cast<ChBody*>(cinfo.modelA->GetContactable()));
        bool okB = cinfo.shapeB->GetMaterial() == BodyMaterial(*static_cast<ChBody*>(cinfo.modelB->GetContactable()));
        if (!okA || !okB)
            num_bad_shapes++;

        float mu = cinfo.shapeA->GetMaterial()->GetSlidingFriction() + cinfo.shapeB->GetMaterial()->GetSlidingFriction();
        if (auto smc = dynamic_cast<ChContactMaterialCompositeSMC*>(mat))
            smc->mu_eff = mu;
        else if (auto nsc = dynamic_cast<ChContactMaterialCompositeNSC*>(mat))
            nsc->sliding_friction = mu;
    }

    std::atomic<int> num_calls{0};
    std::atomic<int> num_bad_shapes{0};
};

class CompositeMaterialTest : public ::testing::TestWithParam<ChContactMethod> {
  protected:
    CompositeMaterialTest();
    ~CompositeMaterialTest() { delete sys; }

    std::shared_ptr<ChContactMaterial> MakeMaterial(int i);
    void CheckContacts(bool with_callback);

    ChSystemMulticore* sys;
    int num_expected_contacts;
};

std::shared_ptr<ChContactMaterial> CompositeMaterialTest::MakeMaterial(int i) {
    if (GetParam() == ChContactMethod::SMC) {
        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetYoungModulus(1e6f * (1 + i));
        mat->SetPoissonRatio(0.2f + 0.02f * i);
        mat->SetFriction(0.1f + 0.1f * i);
        mat->SetRollingFriction(0.01f * i);
        mat->SetSpinningFriction(0.02f * i);
        mat->SetRestitution(0.1f + 0.05f * i);
        mat->SetAdhesion(0.5f * i);
        mat->SetAdhesionMultDMT(0.1f * i);
        mat->SetAdhesionSPerko(0.2f * i);
        mat->SetKn(1e5f * (1 + i));
        mat->SetKt(2e4f * (1 + i));
        mat->SetGn(40.0f + i);
        mat->SetGt(20.0f + i);
        return mat;
    }

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.1f + 0.1f * i);
    mat->SetRollingFriction(0.01f * i);
    mat->SetSpinningFriction(0.02f * i);
    mat->SetRestitution(0.1f + 0.05f * i);
    mat->SetCohesion(0.5f * i);
    mat->SetCompliance(1e-5f * i);
    mat->SetComplianceT(2e-5f * i);
    mat->SetComplianceRolling(3e-5f * i);
    mat->SetComplianceSpinning(4e-5f * i);
    return mat;
}

CompositeMaterialTest::CompositeMaterialTest() : sys(nullptr) {
    if (GetParam() == ChContactMethod::SMC) {
        auto sysSMC = new ChSystemMulticoreSMC;
        sysSMC->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
        sysSMC->GetSettings()->solver.use_material_properties = false;
        sys = sysSMC;
    } else {
        auto sysNSC = new ChSystemMulticoreNSC;
        sysNSC->GetSettings()->solver.solver_mode = SolverMode::SPINNING;
        sysNSC->GetSettings()->collision.collision_envelope = 0.01;
        sys = sysNSC;
    }
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys->SetNumThreads(4);

    double radius = 0.1;
    double mass = 1;
    double pen = 1e-4;

    // Ground box (top surface at z = 0) with material 0
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    utils::AddBoxGeometry(ground.get(), MakeMaterial(0), ChVector3d(4, 4, 0.2), ChVector3d(0, 0, -0.1));
    sys->AddBody(ground);

    // Row of spheres on the ground, each with its own material, plus one sphere stacked on the first one
    int num_row = 4;
    for (int i = 0; i <= num_row; i++) {
        ChVector3d pos = (i < num_row) ? ChVector3d(0.5 * i, 0, radius - pen) : ChVector3d(0, 0, 3 * radius - 2 * pen);
        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
        ball->SetPos(pos);
        ball->EnableCollision(true);
        utils::AddSphereGeometry(ball.get(), MakeMaterial(1 + i), radius);
        sys->AddBody(ball);
    }

    num_expected_contacts = num_row + 1;
}

void CompositeMaterialTest::CheckContacts(bool with_callback) {
    std::shared_ptr<TestAddContactCallback> callback;
    if (with_callback) {
        callback = chrono_types::make_shared<TestAddContactCallback>();
        sys->GetContactContainer()->RegisterAddContactCallback(callback);
    }

    sys->DoStepDynamics(1e-4);

    auto dm = sys->data_manager;
    int num_contacts = (int)dm->cd_data->num_rigid_contacts;
    ASSERT_EQ(num_contacts, num_expected_contacts);
    if (with_callback) {
        ASSERT_EQ(callback->num_calls.load(), num_contacts);
        ASSERT_EQ(callback->num_bad_shapes.load(), 0);
    }

    auto strategy = dm->composition_strategy.get();
    const auto& bodies = sys->GetBodies();
    const auto& host = dm->host_data;

    for (int i = 0; i < num_contacts; i++) {
        auto b1 = dm->cd_data->bids_rigid_rigid[i].x;
        auto b2 = dm->cd_data->bids_rigid_rigid[i].y;
        auto mat1 = BodyMaterial(*bodies[b1]);
        auto mat2 = BodyMaterial(*bodies[b2]);
        float mu_callback = mat1->GetSlidingFriction() + mat2->GetSlidingFriction();

        if (GetParam() == ChContactMethod::SMC) {
            ChContactMaterialCompositeSMC cmat(strategy, std::static_pointer_cast<ChContactMaterialSMC>(mat1), std::static_pointer_cast<ChContactMaterialSMC>(mat2));
            ASSERT_EQ(host.fric_rigid_rigid[i].x, with_callback ? mu_callback : cmat.mu_eff);
            ASSERT_EQ(host.fric_rigid_rigid[i].y, cmat.muRoll_eff);
            ASSERT_EQ(host.fric_rigid_rigid[i].z, cmat.muSpin_eff);
            ASSERT_EQ(host.modulus_rigid_rigid[i].x, cmat.E_eff);
            ASSERT_EQ(host.modulus_rigid_rigid[i].y, cmat.G_eff);
            ASSERT_EQ(host.adhesion_rigid_rigid[i].x, cmat.adhesion_eff);
            ASSERT_EQ(host.adhesion_rigid_rigid[i].y, cmat.adhesionMultDMT_eff);
            ASSERT_EQ(host.adhesion_rigid_rigid[i].z, cmat.adhesionSPerko_eff);
            ASSERT_EQ(host.cr_rigid_rigid[i], cmat.cr_eff);
            ASSERT_EQ(host.smc_rigid_rigid[i].x, cmat.kn);
            ASSERT_EQ(host.smc_rigid_rigid[i].y, cmat.kt);
            ASSERT_EQ(host.smc_rigid_rigid[i].z, cmat.gn);
            ASSERT_EQ(host.smc_rigid_rigid[i].w, cmat.gt);
        } else {
            ChContactMaterialCompositeNSC cmat(strategy, std::static_pointer_cast<ChContactMaterialNSC>(mat1), std::static_pointer_cast<ChContactMaterialNSC>(mat2));
            ASSERT_EQ(host.fric_rigid_rigid[i].x, with_callback ? mu_callback : cmat.sliding_friction);
            ASSERT_EQ(host.fric_rigid_rigid[i].y, cmat.rolling_friction);
            ASSERT_EQ(host.fric_rigid_rigid[i].z, cmat.spinning_friction);
            ASSERT_EQ(host.coh_rigid_rigid[i], cmat.cohesion);
            ASSERT_EQ(host.compliance_rigid_rigid[i].x, cmat.compliance);
            ASSERT_EQ(host.compliance_rigid_rigid[i].y, cmat.complianceT);
            ASSERT_EQ(host.compliance_rigid_rigid[i].z, cmat.complianceRoll);
            ASSERT_EQ(host.compliance_rigid_rigid[i].w, cmat.complianceSpin);
        }
    }
}

TEST_P(CompositeMaterialTest, per_contact_data) {
    CheckContacts(false);
}

TEST_P(CompositeMaterialTest, add_contact_callback) {
    CheckContacts(true);
}

INSTANTIATE_TEST_SUITE_P(ChronoMulticore, CompositeMaterialTest, ::testing::Values(ChContactMethod::NSC, ChContactMethod::SMC));
