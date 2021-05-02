// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // needed for tf2::Matrix3x3

#include "competition.h"
#include <string>
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac2021");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.bin3_);

    part part1, part2;


    part1.type = "assembly_battery_blue";
    part1.frame = "logical_camera_bins0_frame";
    part1.part_frame_name = "logical_camera_bins0_assembly_battery_blue_1_frame";
    // the pose below is in the logical camera frame
    part1.pose.position.x = 1.05064327909;
    part1.pose.position.y = 0.499451077299;
    part1.pose.position.z = 0.26521618141;
    part1.pose.orientation.x = -0.706584046467;
    part1.pose.orientation.y = -0.00469802700285;
    part1.pose.orientation.z = 0.707608358935;
    part1.pose.orientation.w = 0.00270632356228;

    // Where to place the part in the tray?
    // Get this information from /ariac/orders
    part part1_in_tray;
    part1_in_tray.type = "assembly_battery_blue";
    part1_in_tray.frame = "kit_tray_3";
    part1_in_tray.pose.position.x = 0.15;
    part1_in_tray.pose.position.y = 0.1;
    part1_in_tray.pose.position.z = 0.0;
    tf2::Quaternion myQuaternion;
    part1_in_tray.pose.orientation.x = 0;
    part1_in_tray.pose.orientation.y = 0;
    part1_in_tray.pose.orientation.z = 0;
    part1_in_tray.pose.orientation.w =  1;
    ROS_WARN_STREAM("[main] part pose in order: " << part1_in_tray.pose);


    part2.type = "assembly_battery_blue";
    part2.frame = "logical_camera_bins0_frame";
    part2.part_frame_name = "logical_camera_bins0_assembly_battery_blue_2_frame";
    // the pose below is in the logical camera frame
    part2.pose.position.x = 1.05145907886;
    part2.pose.position.y = 0.498020050184;
    part2.pose.position.z = 0.464485324625;
    part2.pose.orientation.x = -0.707156588729;
    part2.pose.orientation.y = 0.00895115822945;
    part2.pose.orientation.z = 0.706991175616;
    part2.pose.orientation.w = 0.00359351990696;

    // Where to place the part in the tray?
    // Get this information from /ariac/orders
    part part2_in_tray;
    part2_in_tray.type = "assembly_battery_blue";
    part2_in_tray.frame = "kit_tray_3";
    part2_in_tray.pose.position.x = -0.15;
    part2_in_tray.pose.position.y = -0.1;
    part2_in_tray.pose.position.z = 0.0;
    part2_in_tray.pose.orientation.x = 0;
    part2_in_tray.pose.orientation.y = 0;
    part2_in_tray.pose.orientation.z = 0;
    part2_in_tray.pose.orientation.w =  1;
    ROS_WARN_STREAM("[main] part pose in order: " << part2_in_tray.pose);


    // part1
    if (!gantry.pickPart(part1))
    {
        gantry.goToPresetLocation(gantry.after_bin3_);
        spinner.stop();
        ros::shutdown();
    }
    gantry.goToPresetLocation(gantry.before_agv3_1_);
    ros::Duration(1.0).sleep();
    gantry.goToPresetLocation(gantry.before_agv3_2_);
    ros::Duration(1.0).sleep();
    gantry.goToPresetLocation(gantry.at_agv3_);
    ros::Duration(1.0).sleep();
    gantry.placePart(part1_in_tray, part1_in_tray.frame);


    ros::Duration(1.0).sleep();

    gantry.goToPresetLocation(gantry.bin3_);
    // part2
    if (!gantry.pickPart(part2))
    {
        gantry.goToPresetLocation(gantry.after_bin3_);
        spinner.stop();
        ros::shutdown();
    }
    gantry.goToPresetLocation(gantry.before_agv3_1_);
    ros::Duration(1.0).sleep();
    gantry.goToPresetLocation(gantry.before_agv3_2_);
    ros::Duration(1.0).sleep();
    gantry.goToPresetLocation(gantry.at_agv3_);
    ros::Duration(1.0).sleep();
    gantry.placePart(part2_in_tray, part2_in_tray.frame);

    AGVControl agv_control(node);
    // // Get the following arguments from the order
    agv_control.sendAGV("as3", "order_0_kitting_shipment_0", "kit_tray_3");
    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}