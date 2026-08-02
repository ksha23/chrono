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
// Demo showing how to publish joint states and accept joint commands for a URDF
// robot loaded with ChParserURDF.
//
// The R2D2 model from the Chrono data directory is used because it exercises
// every joint type the handlers support: continuous (wheels, head swivel),
// revolute (grippers) and prismatic (gripper extension).
//
// Topics (node name is "chrono_ros_node"):
//   /robot_description   std_msgs/msg/String        the URDF, for RViz / robot_state_publisher
//   /joint_states        sensor_msgs/msg/JointState published state of all 8 movable joints
//   /joint_commands      sensor_msgs/msg/JointState commands applied to the motors
//   /clock               rosgraph_msgs/msg/Clock    simulation clock
//   /tf                  tf2_msgs/msg/TFMessage     link transforms
//
// Try it (from a second shell, after sourcing /opt/ros/jazzy/setup.bash):
//   ros2 topic echo /joint_states
//   ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
//       "{name: ['head_swivel'], position: [1.0]}"
//   ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
//       "{name: ['left_front_wheel_joint'], velocity: [3.0]}"
//
// =============================================================================

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_parsers/ChParserURDF.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/mbs/ChROSJointCommandHandler.h"
#include "chrono_ros/handlers/mbs/ChROSJointStateHandler.h"
#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::parsers;

// =============================================================================

// Simulation settings
double step_size = 1e-3;
double time_end = 60;

// Joints driven with SPEED actuation (everything else uses POSITION actuation)
const std::vector<std::string> speed_joints = {
    "left_front_wheel_joint",   //
    "left_back_wheel_joint",    //
    "right_front_wheel_joint",  //
    "right_back_wheel_joint"    //
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // The demo runs headless by default so it can be driven from a terminal; pass --vis to open a
    // run-time visualization window (only if Chrono was built with Irrlicht or VSG).
    bool use_vis = false;
    std::string robot_urdf = GetChronoDataFile("robot/r2d2/r2d2.urdf");
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--vis")
            use_vis = true;
        else if (arg == "--duration" && i + 1 < argc)
            time_end = std::atof(argv[++i]);
        else if (arg == "--urdf" && i + 1 < argc)
            robot_urdf = argv[++i];
    }

    // ------------------------------------------------------------------------
    // Create the Chrono system and load the URDF robot
    // ------------------------------------------------------------------------

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration({0, 0, -9.81});
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    std::cout << "Loading URDF: " << robot_urdf << std::endl;
    ChParserURDF robot(robot_urdf);

    robot.SetRootInitPose(ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT));

    // Mirror what a robotics user would do: actuate every eligible joint, using POSITION control
    // for the articulated joints and SPEED control for the wheels. ChROSJointCommandHandler
    // detects the resulting motor types and routes JointState.position / .velocity accordingly.
    robot.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);
    for (const auto& joint : speed_joints)
        robot.SetJointActuationType(joint, ChParserURDF::ActuationType::SPEED);

    robot.PopulateSystem(sys);

    // Keep the base fixed: this demo is about joint control, not locomotion.
    robot.GetRootChBody()->SetFixed(true);

    // ------------------------------------------------------------------------
    // Create the ROS manager and handlers
    // ------------------------------------------------------------------------

    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Simulation clock on /clock
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    // URDF string on /robot_description (transient local, so RViz can subscribe late)
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSRobotModelHandler>(robot));

    // Link transforms on /tf
    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(30);
    tf_handler->AddURDF(robot);
    ros_manager->RegisterHandler(tf_handler);

    // Joint states on /joint_states at 50 Hz
    auto joint_state_handler = chrono_types::make_shared<ChROSJointStateHandler>(50, "/joint_states");
    int num_states = joint_state_handler->AddURDF(robot);
    ros_manager->RegisterHandler(joint_state_handler);

    // Joint commands on /joint_commands, applied on every simulation step
    auto joint_command_handler = chrono_types::make_shared<ChROSJointCommandHandler>(0, "/joint_commands");
    int num_commands = joint_command_handler->AddURDF(robot);
    ros_manager->RegisterHandler(joint_command_handler);

    ros_manager->Initialize();

    std::cout << "\nPublishing " << num_states << " joint states on /joint_states:" << std::endl;
    for (const auto& name : joint_state_handler->GetJointNames())
        std::cout << "  " << name << std::endl;
    std::cout << "\nAccepting commands for " << num_commands << " motors on /joint_commands:" << std::endl;
    for (const auto& name : joint_command_handler->GetJointNames())
        std::cout << "  " << name << std::endl;
    const auto mimic_names = robot.GetMimicJointNames();
    if (!mimic_names.empty()) {
        std::cout << "\nURDF <mimic> joints (driven by the parser, not commandable):" << std::endl;
        for (const auto& name : mimic_names)
            std::cout << "  " << name << std::endl;
    }
    std::cout << "\nTry:\n"
              << "  ros2 topic echo /joint_states\n"
              << "  ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
                 "\"{name: ['head_swivel'], position: [1.0]}\"\n"
              << std::endl;

    // ------------------------------------------------------------------------
    // Optional run-time visualization
    // ------------------------------------------------------------------------

    std::shared_ptr<ChVisualSystem> vis;
    if (use_vis) {
#if defined(CHRONO_VSG)
        auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
        vis_vsg->AttachSystem(&sys);
        vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
        vis_vsg->SetWindowTitle("Chrono::ROS joint state / command demo");
        vis_vsg->AddCamera(ChVector3d(2, 2, 1), ChVector3d(0, 0, 0.5));
        vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
        vis_vsg->Initialize();
        vis = vis_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis_irr->AttachSystem(&sys);
        vis_irr->SetCameraVertical(CameraVerticalDir::Z);
        vis_irr->SetWindowSize(1200, 800);
        vis_irr->SetWindowTitle("Chrono::ROS joint state / command demo");
        vis_irr->Initialize();
        vis_irr->AddLogo();
        vis_irr->AddSkyBox();
        vis_irr->AddCamera(ChVector3d(2, 2, 1), ChVector3d(0, 0, 0.5));
        vis_irr->AddTypicalLights();
        vis = vis_irr;
#else
        std::cout << "No run-time visualization module available; running headless." << std::endl;
#endif
    }

    // ------------------------------------------------------------------------
    // Simulation loop
    // ------------------------------------------------------------------------

    const auto joint_names = joint_state_handler->GetJointNames();
    double next_report = 0;

    ChRealtimeStepTimer realtime_timer;
    while (sys.GetChTime() < time_end) {
        if (vis) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        double time = sys.GetChTime();

        if (!ros_manager->Update(time, step_size))
            break;

        sys.DoStepDynamics(step_size);

        // Echo the joint positions to stdout once a second so the effect of a command is visible
        // in the terminal running the demo as well as on the ROS topic.
        if (time >= next_report) {
            next_report += 1.0;
            std::cout << "t = " << std::fixed << std::setprecision(2) << time
                      << "  commands applied: " << joint_command_handler->GetNumCommandsApplied() << "  |";
            for (const auto& name : joint_names) {
                double pos = 0, vel = 0, eff = 0;
                joint_state_handler->GetJointState(name, pos, vel, eff);
                std::cout << " " << name << "=" << std::setprecision(4) << pos;
            }
            std::cout << std::endl;
        }

        realtime_timer.Spin(step_size);
    }

    return 0;
}
