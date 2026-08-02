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
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Handler that publishes sensor_msgs/msg/JointState for a set of Chrono joints
// and/or motors.
//
// =============================================================================

#include "chrono_ros/handlers/mbs/ChROSJointStateHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"

#include <iostream>

namespace chrono {
namespace ros {

ChROSJointStateHandler::ChROSJointStateHandler(double update_rate, const std::string& topic_name)
    : ChROSHandler(update_rate), m_topic_name(topic_name), m_frame_id("") {}

bool ChROSJointStateHandler::Initialize(ChROSBridge& bridge) {
    if (m_joints.empty()) {
        std::cerr << "WARNING: ChROSJointStateHandler has no joints registered; nothing meaningful will be published "
                     "on \""
                  << m_topic_name << "\"." << std::endl;
    }

    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/JointState");
    return true;
}

bool ChROSJointStateHandler::AddMotor(std::shared_ptr<ChLinkMotor> motor, const std::string& name) {
    return AddJoint(motor, name);
}

bool ChROSJointStateHandler::AddJoint(std::shared_ptr<ChLinkBase> joint, const std::string& name) {
    if (!joint) {
        std::cerr << "ChROSJointStateHandler::AddJoint: null link (name \"" << name << "\")." << std::endl;
        return false;
    }

    JointEntry entry;
    entry.name = name.empty() ? joint->GetName() : name;

    if (entry.name.empty()) {
        std::cerr << "ChROSJointStateHandler::AddJoint: link has no name and none was provided." << std::endl;
        return false;
    }

    // Dispatch on the concrete Chrono link type once, at registration time, so the per-tick read
    // does not pay for repeated dynamic casts.
    if (auto motor_rot = std::dynamic_pointer_cast<ChLinkMotorRotation>(joint)) {
        entry.kind = JointKind::MOTOR_ROTATION;
        entry.motor_rot = motor_rot;
    } else if (auto motor_lin = std::dynamic_pointer_cast<ChLinkMotorLinear>(joint)) {
        entry.kind = JointKind::MOTOR_LINEAR;
        entry.motor_lin = motor_lin;
    } else if (auto revolute = std::dynamic_pointer_cast<ChLinkLockRevolute>(joint)) {
        entry.kind = JointKind::LOCK_REVOLUTE;
        entry.lock = revolute;
    } else if (auto prismatic = std::dynamic_pointer_cast<ChLinkLockPrismatic>(joint)) {
        entry.kind = JointKind::LOCK_PRISMATIC;
        entry.lock = prismatic;
    } else {
        std::cerr << "ChROSJointStateHandler::AddJoint: link \"" << entry.name
                  << "\" is not a motor nor a revolute/prismatic joint; skipping." << std::endl;
        return false;
    }

    m_joints.push_back(entry);
    return true;
}

#ifdef CHRONO_HAS_URDF
int ChROSJointStateHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();
    if (!model) {
        std::cerr << "ChROSJointStateHandler::AddURDF: URDF model tree is empty." << std::endl;
        return 0;
    }

    int num_added = 0;
    // model->joints_ is a std::map keyed on joint name, so iteration order is deterministic.
    for (const auto& kv : model->joints_) {
        const auto& joint = kv.second;
        if (!joint)
            continue;

        // Only the joint types that have exactly one degree of freedom belong in a JointState
        // message. Fixed / floating / planar joints are skipped, matching URDF conventions.
        switch (joint->type) {
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS:
            case urdf::Joint::PRISMATIC:
                break;
            default:
                continue;
        }

        auto link = parser.GetChLink(joint->name);
        if (!link) {
            // The joint may have been discarded by the parser (e.g. massless link merging).
            continue;
        }

        if (AddJoint(link, joint->name))
            num_added++;
    }

    return num_added;
}
#endif

std::vector<std::string> ChROSJointStateHandler::GetJointNames() const {
    std::vector<std::string> names;
    names.reserve(m_joints.size());
    for (const auto& joint : m_joints)
        names.push_back(joint.name);
    return names;
}

bool ChROSJointStateHandler::GetJointState(const std::string& name,
                                           double& position,
                                           double& velocity,
                                           double& effort) const {
    for (const auto& joint : m_joints) {
        if (joint.name == name) {
            ReadJoint(joint, position, velocity, effort);
            return true;
        }
    }
    return false;
}

void ChROSJointStateHandler::ReadJoint(const JointEntry& joint, double& position, double& velocity, double& effort) {
    switch (joint.kind) {
        case JointKind::MOTOR_ROTATION:
            position = joint.motor_rot->GetMotorAngle();
            velocity = joint.motor_rot->GetMotorAngleDt();
            effort = joint.motor_rot->GetMotorTorque();
            break;
        case JointKind::MOTOR_LINEAR:
            position = joint.motor_lin->GetMotorPos();
            velocity = joint.motor_lin->GetMotorPosDt();
            effort = joint.motor_lin->GetMotorForce();
            break;
        case JointKind::LOCK_REVOLUTE:
            // ChLinkLockRevolute leaves rotation about the link Z axis free (see ChLinkLock.cpp
            // BuildLink for Type::REVOLUTE). GetRelAngleAxis() is the relative rotation vector,
            // so its z component is the signed joint angle.
            position = joint.lock->GetRelAngleAxis().z();
            velocity = joint.lock->GetRelativeAngVel().z();
            effort = 0;
            break;
        case JointKind::LOCK_PRISMATIC:
            // ChLinkLockPrismatic leaves translation along the link Z axis free.
            position = joint.lock->GetRelCoordsys().pos.z();
            velocity = joint.lock->GetRelCoordsysDt().pos.z();
            effort = 0;
            break;
    }
}

void ChROSJointStateHandler::Tick(double time) {
    if (m_joints.empty())
        return;

    const size_t n = m_joints.size();
    m_names.resize(n);
    m_positions.resize(n);
    m_velocities.resize(n);
    m_efforts.resize(n);

    for (size_t i = 0; i < n; i++) {
        m_names[i] = m_joints[i].name;
        ReadJoint(m_joints[i], m_positions[i], m_velocities[i], m_efforts[i]);
    }

    auto msg = m_publisher->NewMessage();
    // Stamp with the Chrono simulation time so the message is consistent with /clock.
    msg.SetTime("header.stamp", time);
    msg.SetString("header.frame_id", m_frame_id);
    msg.SetStringArray("name", m_names);
    msg.SetBlob("position", m_positions.data(), n);
    msg.SetBlob("velocity", m_velocities.data(), n);
    msg.SetBlob("effort", m_efforts.data(), n);

    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
