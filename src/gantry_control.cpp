#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <string>

/**
 * @brief Construct a new Gantry Control:: Gantry Control object
 * 
 * @param node 
 */
GantryControl::GantryControl(ros::NodeHandle &node) : node_("/ariac/gantry"),
                                                      planning_group_("/ariac/gantry/robot_description"),
                                                      gantry_arm_options_("gantry_arm", planning_group_, node_),
                                                      gantry_torso_options_("gantry_torso", planning_group_, node_),
                                                      gantry_full_options_("gantry_full", planning_group_, node_),
                                                      gantry_full_group_(gantry_full_options_),
                                                      gantry_arm_group_(gantry_arm_options_),
                                                      gantry_torso_group_(gantry_torso_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

////////////////////////////
void GantryControl::init()
{
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();

    ROS_INFO_NAMED("init", "Gantry Full planning frame: %s", gantry_full_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Gantry Arm planning frame: %s", gantry_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Gantry Torso planning frame: %s", gantry_torso_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", gantry_arm_group_.getEndEffectorLink().c_str());

    gantry_arm_group_.setPoseReferenceFrame("world");

    // joint positions to go to start location
    home_.gantry_torso_location = {0, 0, 0};
    home_.gantry_arm_location = {0, -0.75, 1.51, -0.88, 1.54, 0.83};

    agv2_.gantry_torso_location = {-0.26, 0, -0.63};
    agv2_.gantry_arm_location = {0, -0.75, 1.51, -0.88, 1.54, 0.83};

    bin3_.gantry_torso_location = {-1.45, -2.43, -PI / 2};
    bin3_.gantry_arm_location = {0, -0.75, 1.51, -0.88, 1.54, 0.83};

    after_bin3_.gantry_torso_location = {-2.34, -2.34, -0.02};
    after_bin3_.gantry_arm_location = {-0.40, -0.32, 1.16, -0.84, 1.54, 0.83};

    before_agv3_1_.gantry_torso_location = {-2.19, 0.09, - PI};
    before_agv3_1_.gantry_arm_location = {-0.40, -0.32, 1.16, -0.84, 1.54, 0.83};

    before_agv3_2_.gantry_torso_location = {0.18, 0.09, - PI};
    before_agv3_2_.gantry_arm_location = {-0.40, -0.32, 1.16, -0.84, 1.54, 0.83};

    at_agv3_.gantry_torso_location = {0.18, 0.63, - PI};
    at_agv3_.gantry_arm_location = {-0.40, -0.32, 1.16, -0.84, 1.54, 0.83};

    // pointers are frequently used to refer to the planning group for improved performance.
    // to start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup *joint_model_group =
        gantry_full_group_.getCurrentState()->getJointModelGroup("gantry_full");

    // Let’s get the current state of the whole gantry robot (torso + arm)
    moveit::core::RobotStatePtr current_state = gantry_full_group_.getCurrentState();

    // Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

    gantry_torso_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    gantry_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);


    // subscribers to some state topics
    joint_states_subscriber_ = node_.subscribe(
        "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    gantry_arm_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/arm/gripper/state", 10, &GantryControl::gantry_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    gantry_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_arm_controller/state", 10, &GantryControl::gantry_arm_controller_state_callback, this);

    while ((current_gantry_controller_state_.joint_names.size() == 0) ||
           (current_gantry_arm_controller_state_.joint_names.size() == 0))
    {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(2).sleep();
    }

    gantry_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
    gantry_gripper_control_client.waitForExistence();


    ROS_INFO("[GantryControl::init] gantry initialized...");
}


////////////////////////////
geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
std::string tray_frame,
std::string part_frame_name)
{
    tf2::Quaternion q_part_orientation(
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w);
    double part_roll, part_pitch, part_yaw;
    tf2::Matrix3x3(q_part_orientation).getRPY(part_roll, part_pitch, part_yaw);
    // ROS_INFO_STREAM("[pickPart] Part pose in tray frame (rpy): "
    //                 << "[" << part_roll
    //                 << " " << part_pitch
    //                 << " " << part_yaw << "]");

    tf2::Quaternion adjusted_yaw;
    // multiply yaw by -1
    // Create this quaternion from roll/pitch/yaw (in radians)
    adjusted_yaw.setRPY(part_roll, part_pitch, -1 * part_yaw);
    adjusted_yaw.normalize();


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::StaticTransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped tfMessage;

    tfMessage.header.frame_id = tray_frame;
    tfMessage.header.stamp = ros::Time::now();
    tfMessage.child_frame_id = "target_"+part_frame_name;
    tfMessage.transform.translation.x = target.position.x;
    tfMessage.transform.translation.y = target.position.y;
    tfMessage.transform.translation.z = target.position.z;
    tfMessage.transform.rotation.x = adjusted_yaw.x();
    tfMessage.transform.rotation.y = adjusted_yaw.y();
    tfMessage.transform.rotation.z = adjusted_yaw.z();
    tfMessage.transform.rotation.w = adjusted_yaw.w();

    // try to broadcast target_frame a few times to make sure
    for (int i = 0; i < 10; ++i)
    {
        tfBroadcaster.sendTransform(tfMessage);
    }

    // tf lookup fails occasionally, this automatically retries the lookup
    unsigned MAX_ATTEMPTS = 5;
    unsigned attempts = 0;
    ros::Duration timeout(5);
    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;

    // ROS_WARN_STREAM("[GantryControl][getTargetWorldPose] Pause for 5s");

    while (attempts < MAX_ATTEMPTS)
    {
        try
        {
            world_target_tf = tfBuffer.lookupTransform(
                "world",
                tfMessage.child_frame_id,
                ros::Time(0),
                timeout);

            ee_target_tf = tfBuffer.lookupTransform(
                tfMessage.child_frame_id,
                "gantry_arm_ee_link",
                ros::Time(0),
                timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        attempts++;
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    ROS_WARN_STREAM("[GantryControl][getTargetWorldPose] world_target =" << world_target);

    return world_target;
}

// ////////////////////////////
bool GantryControl::pickPart(part part)
{
    auto current_pose = gantry_arm_group_.getCurrentPose().pose;
    // Activate gripper
    activateGripper("gantry");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    unsigned MAX_ATTEMPTS = 10;
    unsigned attempts = 0;
    ros::Duration timeout(0.1);
    geometry_msgs::TransformStamped world_part_tf;
    geometry_msgs::TransformStamped camera_ee_tf;

    ROS_INFO_STREAM("TF Listener started");
    while (attempts < MAX_ATTEMPTS)
    {
        try
        {
            world_part_tf = tfBuffer.lookupTransform(
                "world",
                part.part_frame_name,
                ros::Time(0),
                timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try
        {
            camera_ee_tf = tfBuffer.lookupTransform(
                part.part_frame_name,
                "gantry_arm_ee_link",  // change this if you are using the right arm
                ros::Time(0),
                timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        attempts++;
    }
    ROS_INFO_STREAM("TF Listener ended");

    if (part.type.size() == 0)
    {
        ROS_INFO_STREAM("Part name is not present");
        return false;
    }

    // build the target pose to move the arm to
    part.pose.position.x = world_part_tf.transform.translation.x;
    part.pose.position.y = world_part_tf.transform.translation.y;
    part.pose.position.z = world_part_tf.transform.translation.z +
                           model_height.at(part.type)-0.003;
    part.pose.orientation.x = camera_ee_tf.transform.rotation.x;
    part.pose.orientation.y = camera_ee_tf.transform.rotation.y;
    part.pose.orientation.z = camera_ee_tf.transform.rotation.z;
    part.pose.orientation.w = camera_ee_tf.transform.rotation.w;

    auto state = getGripperState("gantry");
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        // move arm to previous position
        gantry_arm_group_.setPoseTarget(part.pose);
        gantry_arm_group_.move();

        auto state = getGripperState("gantry");

        while (!state.attached)
        {
            state = getGripperState("gantry");
            gantry_arm_group_.setPoseTarget(current_pose);
            gantry_arm_group_.move();
            part.pose.position.z = part.pose.position.z - 0.001;
            gantry_arm_group_.setPoseTarget(part.pose);
            gantry_arm_group_.move();
        }

        // move arm to previous position
        ROS_INFO_STREAM("[Gripper] = object attached");
        gantry_arm_group_.setPoseTarget(current_pose);
        gantry_arm_group_.move();
        goToPresetLocation(after_bin3_);
        return true;
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}


// ////////////////////////////
void GantryControl::placePart(part part, std::string tray_frame)
{
    auto target_pose_in_world = getTargetWorldPose(part.pose, tray_frame, part.part_frame_name);
    target_pose_in_world.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    gantry_arm_group_.setPoseTarget(target_pose_in_world);
    gantry_arm_group_.setMaxVelocityScalingFactor(0.1);
    gantry_arm_group_.move();

    deactivateGripper("gantry");
    goToPresetLocation(before_agv3_2_);
    goToPresetLocation(before_agv3_1_);
}

//////////////////////////////
void GantryControl::goToPresetLocation(PresetLocation location)
{
    // gantry torso
    joint_group_positions_.at(0) = location.gantry_torso_location.at(0);
    joint_group_positions_.at(1) = location.gantry_torso_location.at(1);
    joint_group_positions_.at(2) = location.gantry_torso_location.at(2);
    // gantry arm
    joint_group_positions_.at(3) = location.gantry_arm_location.at(0);
    joint_group_positions_.at(4) = location.gantry_arm_location.at(1);
    joint_group_positions_.at(5) = location.gantry_arm_location.at(2);
    joint_group_positions_.at(6) = location.gantry_arm_location.at(3);
    joint_group_positions_.at(7) = location.gantry_arm_location.at(4);
    joint_group_positions_.at(8) = location.gantry_arm_location.at(5);


    gantry_full_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (gantry_full_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        gantry_full_group_.move();
}

// ////////////////////////////
void GantryControl::activateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    gantry_gripper_control_client.call(srv);
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
void GantryControl::deactivateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    gantry_gripper_control_client.call(srv);

    if (arm_name == "gantry")
    {
        gantry_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

// ////////////////////////////
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name)
{
    if (arm_name == "gantry")
    {
        return current_gantry_gripper_state_;
    }
}


// ////////////////////////////
void GantryControl::gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    current_gantry_gripper_state_ = *gripper_state_msg;
}

// ////////////////////////////
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    if (joint_state_msg->position.size() == 0)
    {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}

// ////////////////////////////
void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    current_gantry_controller_state_ = *msg;
}


// ////////////////////////////
void GantryControl::gantry_arm_controller_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    current_gantry_arm_controller_state_ = *msg;
}
