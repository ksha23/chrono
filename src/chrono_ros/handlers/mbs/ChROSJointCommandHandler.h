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

#ifndef CH_ROS_JOINT_COMMAND_HANDLER_H
#define CH_ROS_JOINT_COMMAND_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChConfigROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/functions/ChFunctionSetpoint.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"

#ifdef CHRONO_HAS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

class ChROSSubscription;

/// @addtogroup ros_mbs_handlers
/// @{

/// Subscribes to sensor_msgs/msg/JointState used as a *command* and applies the commanded values to
/// the corresponding Chrono motors. Symmetric with ChROSJointStateHandler, which publishes the same
/// message type.
///
/// ACTUATION MODES
/// ChParserURDF::SetJointActuationType creates ChLinkMotorRotationAngle / ChLinkMotorRotationSpeed /
/// ChLinkMotorRotationTorque (and the linear equivalents) for POSITION / SPEED / FORCE. This handler
/// mirrors those three modes: the mode of each motor is detected from its concrete Chrono type when
/// it is registered, and the matching field of the incoming JointState is used:
///   POSITION -> JointState.position, SPEED -> JointState.velocity, FORCE -> JointState.effort.
/// If the command message does not carry the field required by a motor's mode, the handler falls
/// back to whichever single field the message did provide, so that a caller who only knows how to
/// fill in `position` can still drive a speed motor. Set SetStrictFieldMatching(true) to disable
/// that fallback and ignore commands that do not populate the expected field.
///
/// sensor_msgs/msg/JointState is used rather than a Chrono-specific message because it is the
/// standard message that carries exactly (name, position, velocity, effort).
class CH_ROS_API ChROSJointCommandHandler : public ChROSHandler {
  public:
    /// Actuation mode of a motor, mirroring ChParserURDF::ActuationType.
    enum class ActuationMode {
        POSITION,  ///< angle [rad] for rotational motors, position [m] for linear motors
        SPEED,     ///< angular speed [rad/s] or linear speed [m/s]
        FORCE      ///< torque [Nm] or force [N]
    };

    /// @param update_rate rate (Hz, sim time) at which received commands are applied; 0 = every
    ///        step, which is what a command handler normally wants.
    /// @param topic_name  topic to subscribe to for sensor_msgs/msg/JointState commands.
    ChROSJointCommandHandler(double update_rate, const std::string& topic_name = "~/input/joint_commands");

    /// Creates the JointState subscription.
    virtual bool Initialize(ChROSBridge& bridge) override;

    /// Register a motor to be driven by joint commands.
    /// The actuation mode is deduced from the concrete motor type. The motor's actuation function is
    /// replaced by a ChFunctionSetpoint unless it already is one.
    /// @param motor the motor (rotational or linear)
    /// @param name  name matched against JointState.name; defaults to the Chrono link name
    /// @return true if the motor was registered
    bool AddMotor(std::shared_ptr<ChLinkMotor> motor, const std::string& name = "");

#ifdef CHRONO_HAS_URDF
    /// Register every actuated joint (i.e. every joint turned into a ChLinkMotor by
    /// ChParserURDF::SetJointActuationType / SetAllJointsActuationType) of a parsed URDF model.
    /// Must be called after ChParserURDF::PopulateSystem().
    /// @return the number of motors registered
    int AddURDF(chrono::parsers::ChParserURDF& parser);
#endif

    /// If true, a command is applied only if the message populated the field matching the motor's
    /// actuation mode. If false (default), a single populated field is used for any mode.
    void SetStrictFieldMatching(bool strict) { m_strict_field_matching = strict; }

    /// Get the names of all motors registered with this handler.
    std::vector<std::string> GetJointNames() const;

    /// Get the number of motors registered with this handler.
    std::size_t GetNumJoints() const { return m_motors.size(); }

    /// Get the number of command messages applied so far (useful for tests and demos).
    uint64_t GetNumCommandsApplied() const { return m_num_commands_applied; }

  protected:
    /// Apply the most recently received command batch, if any.
    virtual void Tick(double time) override;

  private:
    struct MotorEntry {
        std::shared_ptr<ChLinkMotor> motor;
        std::shared_ptr<ChFunctionSetpoint> setpoint;
        ActuationMode mode;
    };

    /// One entry of a received sensor_msgs/msg/JointState. The three value arrays of that message
    /// are independently optional, so record which of them this entry actually carried.
    struct CommandEntry {
        std::string name;
        double position = 0;
        double velocity = 0;
        double effort = 0;
        bool has_position = false;
        bool has_velocity = false;
        bool has_effort = false;
    };

    /// Apply one command entry to the corresponding motor. Returns true if a setpoint was written.
    bool ApplyCommand(const CommandEntry& entry, double time);

    const std::string m_topic_name;              ///< topic to subscribe to
    std::map<std::string, MotorEntry> m_motors;  ///< registered motors, keyed on joint name
    std::vector<std::string> m_motor_order;      ///< registration order, for GetJointNames()
    std::shared_ptr<ChROSSubscription> m_subscription;

    bool m_strict_field_matching = true;  ///< see SetStrictFieldMatching()
    uint64_t m_num_commands_applied = 0;   ///< number of command messages applied

    // Most recent command batch (written in the subscription callback, applied in Tick; both run on
    // the simulation thread inside ChROSManager::Update(), so no lock). Applied once per message,
    // since ChFunctionSetpoint holds its value until the next SetSetpoint.
    std::vector<CommandEntry> m_pending;
    bool m_have_pending = false;
};

/// @} ros_mbs_handlers

}  // namespace ros
}  // namespace chrono

#endif
