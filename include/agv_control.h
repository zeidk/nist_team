#ifndef AGVCONTROL_H
#define AGVCONTROL_H

#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "utils.h"

class AGVControl
{
public:
    explicit AGVControl(ros::NodeHandle &);
    bool isAGVReady(std::string tray_name);
    bool sendAGV(std::string assembly_station, std::string shipment_type, std::string kit_tray);
    void agv3_state_callback(const std_msgs::String & msg);

private:
    ros::ServiceClient agv3_client;
    ros::Subscriber agv3_state_subscriber_;
    bool agv3_ready;
};

#endif  // AGV_CONTROL_H
