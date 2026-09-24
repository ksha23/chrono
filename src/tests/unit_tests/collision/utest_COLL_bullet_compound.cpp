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
// Unit test for the Bullet compound-vs-shape collision algorithm.
//
// A connected triangle mesh is injected in Bullet as a compound of triangle shapes with a dynamic AABB tree, which the
// compound collision algorithm uses to cull the children tested against the other shape. This test places a wheel-like
// mesh on a large box (and next to a small box) at several orientations and checks that the contacts found with the
// AABB tree are exactly the contacts found by the unculled path (the same compound without a tree, where every child
// is tested).
//
// =============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorld.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionObject.h"

#include "gtest/gtest.h"

using namespace chrono;

// Open tube (tire tread) around the y axis, as a connected mesh.
static std::shared_ptr<ChTriangleMeshConnected> CreateTube(double radius, double width, int n_around, int n_across) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    for (int j = 0; j <= n_across; j++) {
        double y = -width / 2 + width * j / n_across;
        for (int i = 0; i < n_around; i++) {
            double a = CH_2PI * i / n_around;
            mesh->m_vertices.push_back(ChVector3d(radius * std::cos(a), y, radius * std::sin(a)));
        }
    }
    for (int j = 0; j < n_across; j++) {
        for (int i = 0; i < n_around; i++) {
            int v00 = j * n_around + i;
            int v01 = j * n_around + (i + 1) % n_around;
            int v10 = v00 + n_around;
            int v11 = v01 + n_around;
            mesh->m_face_v_indices.push_back(ChVector3i(v00, v01, v11));
            mesh->m_face_v_indices.push_back(ChVector3i(v00, v11, v10));
        }
    }
    return mesh;
}

// Contact record, compared exactly.
using ContactRecord = std::array<double, 10>;

class ContactCollector : public ChCollisionSystem::NarrowphaseCallback {
  public:
    virtual bool OnNarrowphase(ChCollisionInfo& cinfo) override {
        contacts.push_back({cinfo.vpA.x(), cinfo.vpA.y(), cinfo.vpA.z(), cinfo.vpB.x(), cinfo.vpB.y(), cinfo.vpB.z(), cinfo.vN.x(), cinfo.vN.y(), cinfo.vN.z(), cinfo.distance});
        return true;
    }
    std::vector<ContactRecord> contacts;
};

struct Scenario {
    ChVector3d wheel_pos;
    ChQuaterniond wheel_rot;
    bool small_box;    // other shape: small box next to the tread (true) or large ground box (false)
    bool wheel_first;  // add the wheel to the system before the other body (swaps body order in Bullet pairs)
};

// Run one collision detection pass and return the sorted contact set.
// If 'unculled' is true, the wheel compound is replaced by a copy without a dynamic AABB tree.
static std::vector<ContactRecord> Collide(const Scenario& sc, bool unculled, int& num_children) {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    auto other =
        sc.small_box ? chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, 1000, true, true, mat) : chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, mat);
    other->SetFixed(true);
    other->SetPos(sc.small_box ? ChVector3d(0.59, 0.05, 0.02) : ChVector3d(0, 0, -0.5));
    other->SetRot(sc.small_box ? QuatFromAngleZ(0.3) : QUNIT);

    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetPos(sc.wheel_pos);
    wheel->SetRot(sc.wheel_rot);
    auto mesh = CreateTube(0.5, 0.3, 256, 8);
    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, mesh, false, false, 0.0);
    wheel->AddCollisionShape(shape);
    wheel->EnableCollision(true);

    if (sc.wheel_first) {
        sys.AddBody(wheel);
        sys.AddBody(other);
    } else {
        sys.AddBody(other);
        sys.AddBody(wheel);
    }

    auto collector = chrono_types::make_shared<ContactCollector>();
    sys.GetCollisionSystem()->RegisterNarrowphaseCallback(collector);
    sys.GetCollisionSystem()->Initialize();  // create the Bullet collision objects

    // find the Bullet object of the wheel (the only one with a compound shape)
    auto bt_sys = std::static_pointer_cast<ChCollisionSystemBullet>(sys.GetCollisionSystem());
    cbtCollisionObject* bt_object = nullptr;
    auto& bt_objects = bt_sys->GetBulletCollisionWorld()->getCollisionObjectArray();
    for (int i = 0; i < bt_objects.size(); i++) {
        if (bt_objects[i]->getCollisionShape()->isCompound())
            bt_object = bt_objects[i];
    }
    EXPECT_TRUE(bt_object != nullptr);
    if (!bt_object)
        return {};
    auto compound = static_cast<cbtCompoundShape*>(bt_object->getCollisionShape());
    EXPECT_TRUE(compound->isCompound());
    EXPECT_TRUE(compound->getDynamicAabbTree() != nullptr);
    num_children = compound->getNumChildShapes();

    cbtCompoundShape flat(false);
    if (unculled) {
        flat.setMargin(compound->getMargin());
        flat.setUserPointer(compound->getUserPointer());
        for (int i = 0; i < compound->getNumChildShapes(); i++)
            flat.addChildShape(compound->getChildTransform(i), compound->getChildShape(i));
        bt_object->setCollisionShape(&flat);
    }

    sys.ComputeCollisions();

    if (unculled)
        bt_object->setCollisionShape(compound);  // restore before the system (and 'flat') are destroyed

    std::vector<ContactRecord> contacts = collector->contacts;
    std::sort(contacts.begin(), contacts.end());
    return contacts;
}

TEST(BulletCollision, CompoundChildCulling) {
    std::vector<Scenario> scenarios;
    // wheel resting on the large box with 1 cm penetration, spun about its axle, tilted and yawed
    for (double spin : {0.0, 0.1, 0.7, 1.3, 2.9}) {
        for (double tilt : {0.0, 0.05, 0.3}) {
            ChQuaterniond q = QuatFromAngleZ(0.4 * spin) * QuatFromAngleX(tilt) * QuatFromAngleY(spin);
            scenarios.push_back({ChVector3d(1.5, -2.0, 0.49), q, false, (tilt == 0.05)});
        }
    }
    // wheel above the ground box (no contact)
    scenarios.push_back({ChVector3d(0, 0, 0.6), QuatFromAngleY(0.7), false, false});
    // small box pressed into the side of the tread
    for (double spin : {0.0, 0.7, 2.9})
        scenarios.push_back({ChVector3d(0, 0, 0), QuatFromAngleY(spin) * QuatFromAngleX(0.1), true, spin > 1});

    int num_touching = 0;
    for (size_t k = 0; k < scenarios.size(); k++) {
        int n_culled = 0;
        int n_flat = 0;
        auto culled = Collide(scenarios[k], false, n_culled);
        auto flat = Collide(scenarios[k], true, n_flat);
        ASSERT_EQ(n_culled, 256 * 8 * 2);
        ASSERT_EQ(n_flat, n_culled);
        ASSERT_EQ(culled.size(), flat.size()) << "scenario " << k;
        for (size_t i = 0; i < culled.size(); i++) {
            for (size_t j = 0; j < culled[i].size(); j++)
                ASSERT_EQ(culled[i][j], flat[i][j]) << "scenario " << k << " contact " << i << " component " << j;
        }
        if (!culled.empty())
            num_touching++;
    }
    // all scenarios except the one with the wheel above ground must produce contacts
    EXPECT_EQ(num_touching, (int)scenarios.size() - 1);
}
