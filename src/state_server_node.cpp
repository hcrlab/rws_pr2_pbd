#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "mongo_msg_db_msgs/Find.h"
#include "rapid_ros/service_client.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "rws_pr2_pbd/state_server.h"
#include "urdf/model.h"

using mongo_msg_db_msgs::Find;
using pr2_pbd::StateServer;

int main(int argc, char** argv) {
  // Set up ROS
  ros::init(argc, argv, "pr2_pbd_state_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;

  // Set up server
  rapid_ros::ServiceClient<Find> find(
      nh.serviceClient<Find>("mongo_msg_db/find"));
  StateServer state_server(find);
  ros::ServiceServer publish_server = nh.advertiseService(
      "/subscribe_pbd_state", &StateServer::ServeSubscription, &state_server);
  ros::waitForShutdown();
  return 0;
}
