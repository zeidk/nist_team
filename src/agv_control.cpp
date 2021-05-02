#include "agv_control.h"
#include <string>
#include <nist_gear/AGVToAssemblyStation.h>

AGVControl::AGVControl(ros::NodeHandle &node)
{
    agv3_client =
            node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv3/submit_shipment");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!agv3_client.exists())
    {
        ROS_INFO("Waiting for the AGV3 to be ready...");
        agv3_client.waitForExistence();
        ROS_INFO("AGV3 is now ready.");
    }

    agv3_state_subscriber_ = node.subscribe(
            "ariac/agv3/state", 10, &AGVControl::agv3_state_callback, this);
}

bool AGVControl::isAGVReady(std::string tray_name)
{
    if (tray_name == "kit_tray_3") return agv3_ready;
    return false;
}


bool AGVControl::sendAGV(std::string assembly_station, std::string shipment_type, std::string tray_name)
{
  nist_gear::AGVToAssemblyStation msg;
  msg.request.shipment_type = shipment_type;
  msg.request.assembly_station_name = assembly_station;

  (tray_name == "kit_tray_3") ? agv3_client.call(msg) : agv3_client.call(msg);

  if (msg.response.success)
  {
    ROS_INFO_STREAM("[agv_control][sendAGV] AGV is taking order: " + msg.request.shipment_type);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[agv_control][sendAGV] Failed to call AGV!");
    return false;
  }
}

void AGVControl::agv3_state_callback(const std_msgs::String &msg)
{
    if (!((msg.data).compare("ready_to_deliver")))
        agv3_ready = true;
    else agv3_ready = false;
}

