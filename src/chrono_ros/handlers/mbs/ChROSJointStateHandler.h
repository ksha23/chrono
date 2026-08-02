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

#ifndef CH_ROS_JOINT_STATE_HANDLER_H
#define CH_ROS_JOINT_STATE_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChConfigROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChLinkBase.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"

#ifdef CHRONO_HAS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_mbs_handlers
/// @{

/// Publishes the state (position / velocity / effort) of a set of Chrono joints or motors as a
/// sensor_msgs/msg/JointState message.
///
/// The standard sensor_msgs/msg/JointState is used rather than a Chrono-specific message: it is
/// what every ROS robotics tool (robot_state_publisher, RViz, MoveIt, ros2_control) already
/// consumes, and it is symmetric with ChROSJointCommandHandler.
///
/// Supported Chrono link types (anything else is rejected by the Add* methods):
/// - ChLinkMotorRotation*: position = motor angle [rad], velocity = [rad/s], effort = torque [Nm]
/// - ChLinkMotorLinear*:   position = motor displacement [m], velocity = [m/s], effort = force [N]
/// - ChLinkLockRevolute:   position/velocity derived from the relative marker frames, effort = 0
/// - ChLinkLockPrismatic:  position/velocity derived from the relative marker frames, effort = 0
///
/// Passive (non-motorized) joints report zero effort: a kinematic constraint carries a reaction
/// load but no actuation effort along its free axis, which is what JointState.effort means.
class CH_ROS_API ChROSJointStateHandler : public ChROSHandler {
  public:
    /// @param update_rate publish rate (Hz, sim time); 0 = every step.
    /// @param topic_name  topic the sensor_msgs/msg/JointState message is published on.
    ChROSJointStateHandler(double update_rate, const std::string& topic_name = "~/output/joint_states");

    /// Creates the JointState publisher.
    virtual bool Initialize(ChROSBridge& bridge) override;

    /// Add a Chrono motor to the set of published joints.
    /// @param motor the motor (rotational or linear)
    /// @param name  name reported in JointState.name; defaults to the Chrono link name
    /// @return true if the motor was added
    bool AddMotor(std::shared_ptr<ChLinkMotor> motor, const std::string& name = "");

    /// Add a Chrono joint (motor or passive revolute/prismatic lock) to the set of published joints.
    /// @param joint the link
    /// @param name  name reported in JointState.name; defaults to the Chrono link name
    /// @return true if the joint was added (false for unsupported link types)
    bool AddJoint(std::shared_ptr<ChLinkBase> joint, const std::string& name = "");

#ifdef CHRONO_HAS_URDF
    /// Add every revolute / continuous / prismatic joint of a parsed URDF model.
    /// Joint names are the URDF joint names, so the published JointState lines up directly with the
    /// robot_description published by ChROSRobotModelHandler.
    /// Must be called after ChParserURDF::PopulateSystem().
    /// @return the number of joints added
    int AddURDF(chrono::parsers::ChParserURDF& parser);
#endif

    /// Set the frame_id used in the header of the published message (default: empty).
    void SetFrameId(const std::string& frame_id) { m_frame_id = frame_id; }

    /// Get the names of all joints registered with this handler, in publication order.
    std::vector<std::string> GetJointNames() const;

    /// Get the number of joints registered with this handler.
    std::size_t GetNumJoints() const { return m_joints.size(); }

    /// Read the current state of a registered joint, using the same conversions as the published
    /// message. Useful for demos and tests that want to cross-check the ROS topic.
    /// @return false if no joint with the given name is registered
    bool GetJointState(const std::string& name, double& position, double& velocity, double& effort) const;

  protected:
    /// Read the state of all registered joints and publish them.
    virtual void Tick(double time) override;

  private:
    /// Kind of Chrono link backing a registered joint; determines how state is read.
    enum class JointKind {
        MOTOR_ROTATION,  ///< ChLinkMotorRotation and derived
        MOTOR_LINEAR,    ///< ChLinkMotorLinear and derived
        LOCK_REVOLUTE,   ///< passive ChLinkLockRevolute
        LOCK_PRISMATIC   ///< passive ChLinkLockPrismatic
    };

    struct JointEntry {
        std::string name;
        JointKind kind;
        std::shared_ptr<ChLinkMotorRotation> motor_rot;
        std::shared_ptr<ChLinkMotorLinear> motor_lin;
        std::shared_ptr<ChLinkLock> lock;
    };

    /// Read position / velocity / effort from the Chrono link backing a registered joint.
    static void ReadJoint(const JointEntry& joint, double& position, double& velocity, double& effort);

    const std::string m_topic_name;    ///< topic to publish on
    std::string m_frame_id;            ///< header.frame_id of the published message
    std::vector<JointEntry> m_joints;  ///< registered joints, in publication order
    std::shared_ptr<ChROSPublisher> m_publisher;

    // Scratch buffers, reused across ticks.
    std::vector<std::string> m_names;
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
    std::vector<double> m_efforts;
};

/// @} ros_mbs_handlers

}  // namespace ros
}  // namespace chrono

#endif
