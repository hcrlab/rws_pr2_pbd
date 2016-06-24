#include "rws_pr2_pbd/state_server.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_pbd_state_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;
  pr2_pbd::StateServer state_server(nh);
  ros::ServiceServer server = nh.advertiseService(
      "/publish_pbd_state", &pr2_pbd::StateServer::ServeAdd, &state_server);
  ros::Rate rate(30);
  while (ros::ok()) {
    state_server.Publish();
    rate.sleep();
  }
  return 0;
}
