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
// Handler that subscribes to joint commands and drives Chrono motors.
//
// =============================================================================

#include <cmath>

#include "chrono_ros/handlers/mbs/ChROSJointCommandHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSSubscription.h"

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include <cstring>
#include <iostream>

namespace chrono {
namespace ros {

ChROSJointCommandHandler::ChROSJointCommandHandler(double update_rate, const std::string& topic_name)
    : ChROSHandler(update_rate), m_topic_name(topic_name) {}

bool ChROSJointCommandHandler::Initialize(ChROSBridge& bridge) {
    if (m_motors.empty()) {
        std::cerr << "WARNING: ChROSJointCommandHandler has no motors registered; commands on \"" << m_topic_name
                  << "\" will be ignored." << std::endl;
    }

    // The callback fires inside ChROSManager::Update() on the simulation thread, the same thread as
    // Tick(), so the pending batch needs no lock.
    m_subscription = bridge.CreateSubscription(  //
        m_topic_name, "sensor_msgs/msg/JointState", [this](const ChROSMessageView& msg) {
            const auto names = msg.GetStringArray("name");

            // position / velocity / effort are independently optional in sensor_msgs/JointState.
            const auto positions = msg.GetBlob("position");
            const auto velocities = msg.GetBlob("velocity");
            const auto efforts = msg.GetBlob("effort");

            m_pending.clear();
            m_pending.reserve(names.size());
            for (size_t i = 0; i < names.size(); i++) {
                CommandEntry entry;
                entry.name = names[i];
                if (i < positions.count) {
                    std::memcpy(&entry.position, positions.data + i * sizeof(double), sizeof(double));
                    entry.has_position = true;
                }
                if (i < velocities.count) {
                    std::memcpy(&entry.velocity, velocities.data + i * sizeof(double), sizeof(double));
                    entry.has_velocity = true;
                }
                if (i < efforts.count) {
                    std::memcpy(&entry.effort, efforts.data + i * sizeof(double), sizeof(double));
                    entry.has_effort = true;
                }
                m_pending.push_back(std::move(entry));
            }
            m_have_pending = !m_pending.empty();
        });

    return true;
}

bool ChROSJointCommandHandler::AddMotor(std::shared_ptr<ChLinkMotor> motor, const std::string& name) {
    if (!motor) {
        std::cerr << "ChROSJointCommandHandler::AddMotor: null motor (name \"" << name << "\")." << std::endl;
        return false;
    }

    const std::string motor_name = name.empty() ? motor->GetName() : name;
    if (motor_name.empty()) {
        std::cerr << "ChROSJointCommandHandler::AddMotor: motor has no name and none was provided." << std::endl;
        return false;
    }

    // Deduce the actuation mode from the concrete motor type. These are exactly the motor types
    // that ChParserURDF creates for ActuationType POSITION / SPEED / FORCE.
    ActuationMode mode;
    if (std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(motor) ||
        std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(motor)) {
        mode = ActuationMode::POSITION;
    } else if (std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(motor) ||
               std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(motor)) {
        mode = ActuationMode::SPEED;
    } else if (std::dynamic_pointer_cast<ChLinkMotorRotationTorque>(motor) ||
               std::dynamic_pointer_cast<ChLinkMotorLinearForce>(motor)) {
        mode = ActuationMode::FORCE;
    } else {
        std::cerr << "ChROSJointCommandHandler::AddMotor: motor \"" << motor_name
                  << "\" has an unsupported type (only angle/position, speed and torque/force motors "
                     "are driven by joint commands); skipping."
                  << std::endl;
        return false;
    }

    if (m_motors.find(motor_name) != m_motors.end()) {
        std::cerr << "ChROSJointCommandHandler::AddMotor: motor \"" << motor_name << "\" is already registered."
                  << std::endl;
        return false;
    }

    // Commands are applied by writing into a ChFunctionSetpoint. Reuse an existing one if the
    // caller already installed it (e.g. the URDF demos do), otherwise install one.
    auto setpoint = std::dynamic_pointer_cast<ChFunctionSetpoint>(motor->GetMotorFunction());
    if (!setpoint) {
        setpoint = chrono_types::make_shared<ChFunctionSetpoint>();
        motor->SetMotorFunction(setpoint);
    }

    MotorEntry entry;
    entry.motor = motor;
    entry.setpoint = setpoint;
    entry.mode = mode;

    m_motors[motor_name] = entry;
    m_motor_order.push_back(motor_name);
    return true;
}

#ifdef CHRONO_HAS_URDF
int ChROSJointCommandHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();
    if (!model) {
        std::cerr << "ChROSJointCommandHandler::AddURDF: URDF model tree is empty." << std::endl;
        return 0;
    }

    int num_added = 0;
    // model->joints_ is a std::map keyed on joint name, so iteration order is deterministic.
    for (const auto& kv : model->joints_) {
        const auto& joint = kv.second;
        if (!joint)
            continue;

        // Skip URDF <mimic> joints: the parser already installed an actuation function that slaves
        // them to another joint, and overwriting it here would silently break the coupling.
        if (joint->mimic && parser.GetMimicJointsEnabled())
            continue;

        // Note: GetChLink + dynamic cast rather than GetChMotor, because GetChMotor logs a warning
        // for every joint that was not marked as actuated.
        auto motor = std::dynamic_pointer_cast<ChLinkMotor>(parser.GetChLink(joint->name));
        if (!motor)
            continue;

        if (AddMotor(motor, joint->name))
            num_added++;
    }

    return num_added;
}
#endif

std::vector<std::string> ChROSJointCommandHandler::GetJointNames() const {
    return m_motor_order;
}

void ChROSJointCommandHandler::Tick(double time) {
    if (!m_have_pending)
        return;
    m_have_pending = false;

    bool any_applied = false;
    for (const auto& entry : m_pending)
        any_applied |= ApplyCommand(entry, time);

    if (any_applied)
        m_num_commands_applied++;
}

bool ChROSJointCommandHandler::ApplyCommand(const CommandEntry& entry, double time) {
    auto it = m_motors.find(entry.name);
    if (it == m_motors.end())
        return false;

    const auto& motor_entry = it->second;

    // Pick the JointState field that matches the motor's actuation mode.
    double value = 0;
    bool have_value = false;
    switch (motor_entry.mode) {
        case ActuationMode::POSITION:
            value = entry.position;
            have_value = entry.has_position;
            break;
        case ActuationMode::SPEED:
            value = entry.velocity;
            have_value = entry.has_velocity;
            break;
        case ActuationMode::FORCE:
            value = entry.effort;
            have_value = entry.has_effort;
            break;
    }

    // Fallback, off by default: if the message did not populate the expected field but populated exactly
    // one other field, use that, so a simple publisher can drive motors of any mode without knowing them.
    //
    // JointState carries parallel arrays, so any message naming both a speed motor and a position motor
    // populates only one of the two fields -- and the fallback then applies e.g. "velocity 0" to the
    // position motor as a position. That is the mixed-mode robot demo_ROS_joints builds. Opt in with
    // SetStrictFieldMatching(false) when every joint in a message shares a mode.
    if (!have_value && !m_strict_field_matching) {
        const int num_populated = entry.has_position + entry.has_velocity + entry.has_effort;
        if (num_populated == 1) {
            if (entry.has_position)
                value = entry.position;
            else if (entry.has_velocity)
                value = entry.velocity;
            else
                value = entry.effort;
            have_value = true;
        }
    }

    if (!have_value)
        return false;

    // A command carrying NaN or an infinity would be latched into the motor function and propagate into
    // the solver, from which nothing recovers.
    if (!std::isfinite(value)) {
        std::cerr << "ChROSJointCommandHandler: ignoring non-finite command for joint \"" << entry.name
                  << "\"" << std::endl;
        return false;
    }

    // SetSetpoint derives a backward-difference derivative from the previous setpoint and the time
    // between them, and a position motor feeds that derivative straight into its velocity-level
    // constraint. Applying a position command once therefore leaves the motor holding both "stay at the
    // target" and "keep moving at whatever rate the last two commands implied" -- so two ordinary
    // commands 10 ms apart wind a joint up to hundreds of rad/s, and a slower command stream parks the
    // joint a fixed step*rate away from where it was told to go, forever.
    //
    // A position setpoint is a position, not a trajectory sample: pin the derivatives to zero. Speed and
    // force motors read only the value, so they are unaffected either way.
    if (motor_entry.mode == ActuationMode::POSITION)
        motor_entry.setpoint->SetSetpointAndDerivatives(value, 0, 0);
    else
        motor_entry.setpoint->SetSetpoint(value, time);
    return true;
}

}  // namespace ros
}  // namespace chrono
