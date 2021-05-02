#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>

#include <ros/ros.h>

#include <nist_gear/VacuumGripperState.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef struct Shipment shipment;
typedef struct Order order;
typedef struct Product product;

const double PI = 3.141592;
const int MAX_PICKING_ATTEMPTS = 3;  // for pickup
const double ABOVE_TARGET = 0.2;  // above target z pos when picking/placing part
const double PICK_TIMEOUT = 4.0;
const double RETRIEVE_TIMEOUT = 2.0;

const double GRIPPER_HEIGHT = 0.01;
const double EPSILON = 0.008;  // for the gripper to firmly touch

const double BIN_HEIGHT = 0.724;
const double TRAY_HEIGHT = 0.755;
const double RAIL_HEIGHT = 0.95;

const double PLANNING_TIME = 20;  // for move_group
const int MAX_EXCHANGE_ATTEMPTS = 6;  // Pulley flip

extern std::string action_state_name[];
extern std::unordered_map<std::string, double> model_height;


/**
 * @brief Struct for preset locations
 * @todo Add new preset locations here
 * 
 */
typedef struct PresetLocation
{
    std::vector<double> gantry_torso_location;
    std::vector<double> gantry_arm_location;
    std::vector<double> kitting_arm_location;
}
home, bin1, bin2, bin3, after_pickup, before_place, at_agv, bin4, bin5, bin6, bin7, bin8, agv1, agv2, agv3, agv4;

/**
 * @brief Struct to store part information
 * 
 */
typedef struct Part
{
  std::string type;  // model type
  geometry_msgs::Pose pose;  // model pose (in frame)
  geometry_msgs::Pose save_pose;
  std::string frame;  // model frame (e.g., "logical_camera_1_frame")
  std::string part_frame_name;
  ros::Time time_stamp;
  std::string id;
  bool faulty;
}
part;

/**
 * @brief Struct to store joint positions for each group
 * 
 */
typedef struct Position
{
    std::vector<double> gantry_torso_position;
    std::vector<double> gantry_arm_position;
    std::vector<double> kitting_arm_position;
}
position;

/**
 * @brief Struct to store shipments
 * 
 */
typedef struct Shipment
{
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
}
shipment;

/**
 * @brief Struct to store products
 * 
 */
typedef struct Product
{
    std::string type;
    geometry_msgs::Pose pose;
    part p;
    geometry_msgs::Pose actual_pose;
    std::string actual_pose_frame;
    std::string agv_id;
    std::string tray;
    std::string arm_name;
}
product;

/**
 * @brief struct to parse and store orders published on /ariac/orders
 * 
 */
typedef struct Order
{
    std::string order_id;
    std::vector<Shipment> shipments;
} order;



#endif