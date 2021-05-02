#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"


class GantryControl
{
  public:
    explicit GantryControl(ros::NodeHandle & node);
    void init();
    bool pickPart(part part);
    void placePart(part part, std::string agv);
    void placePartWorldPose(part part);

    // Send command message to robot controller
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv, std::string part_frame_name);
    // geometry_msgs::Pose convertToWorld(geometry_msgs::Pose part_pose, std::string agv);
    geometry_msgs::Pose convert_to_world(part part);

    // preset locations;
    after_pickup after_bin3_;
    before_place before_agv3_1_, before_agv3_2_;
    at_agv at_agv3_;
    home home_;
    bin1 bin1_;
    bin2 bin2_;
    bin3 bin3_;
    bin4 bin4_;
    bin5 bin5_;
    bin6 bin6_;
    bin7 bin7_;
    bin8 bin8_;
    agv1 agv1_;
    agv2 agv2_;
    agv3 agv3_;
    agv4 agv4_;


  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_full_options_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_torso_options_;
    moveit::planning_interface::MoveGroupInterface gantry_full_group_;
    moveit::planning_interface::MoveGroupInterface gantry_arm_group_;
    moveit::planning_interface::MoveGroupInterface gantry_torso_group_;

    sensor_msgs::JointState current_joint_states_;
    nist_gear::VacuumGripperState current_gantry_gripper_state_;
    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_gantry_arm_controller_state_;

    ros::Publisher gantry_torso_joint_trajectory_publisher_;
    ros::Publisher gantry_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber gantry_arm_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber gantry_arm_controller_state_subscriber_;

    ros::ServiceClient gantry_gripper_control_client;
    geometry_msgs::TransformStamped tray_transforms_ [2];

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
};

#endif  // GANTRY_CONTROL_H
