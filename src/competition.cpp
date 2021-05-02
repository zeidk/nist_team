#include "competition.h"
#include "utils.h"
#include<string>

#include <std_srvs/Trigger.h>


////////////////////////
Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
  node_ = node;
}

////////////////////////
void Competition::init()
{
  // Subscribe to the '/ariac/current_score' topic.
  double time_called = ros::Time::now().toSec();
  competition_start_time_ = ros::Time::now().toSec();

  // Subscribe to the '/ariac/competition_state' topic.
  ROS_INFO("Subscribe to the /ariac/competition_state topic...");
  competition_state_subscriber_ = node_.subscribe(
    "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

  // Subscribe to the '/clock' topic.
  ROS_INFO("Subscribe to the /clock...");
  competition_clock_subscriber_ = node_.subscribe(
    "/clock", 10, &Competition::competition_clock_callback, this);

    ROS_INFO("Subscribe to the /orders...");
    orders_subscriber_ = node_.subscribe(
            "/ariac/orders", 10, &Competition::order_callback, this);

  startCompetition();

 
}

////////////////////////
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "done" && competition_state_ != "done")
  {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

////////////////////////
void Competition::order_callback(const nist_gear::Order::ConstPtr &order_msg)
{
  ROS_INFO_STREAM("Received order:\n"
                  << *order_msg);
  received_orders_.push_back(*order_msg);

  order new_order;
  new_order.order_id = order_msg->order_id;

  order_list_.push_back(new_order);
}

////////////////////////
void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg)
{
  competition_clock_ = msg->clock;
}

////////////////////////
void Competition::processOrder()
{
  auto current_order = order_list_.front();
  auto current_shipment = current_order.shipments.front();//--change this line to handle multiple shipments
  auto product_list = current_shipment.products;

  for (const auto &product: product_list)
  {
    product_list.push_back(product);
  }
}

////////////////////////
void Competition::startCompetition()
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready...");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}


////////////////////////
void Competition::endCompetition()
{
  ros::ServiceClient end_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

  if (!end_client.exists())
  {
    ROS_INFO("Waiting for end_competition...");
    end_client.waitForExistence();
    ROS_INFO("end_competition is now ready.");
  }
  ROS_INFO("Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition ended");
  }
}


////////////////////////
double Competition::getStartTime()
{
  return competition_start_time_;
}

////////////////////////
double Competition::getClock()
{
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("Competition time spent =" << time_spent);
  return time_spent;
}

////////////////////////
std::string Competition::getCompetitionState()
{
  return competition_state_;
}
