#ifndef _PR2_PBD_STATE_SERVER_H_
#define _PR2_PBD_STATE_SERVER_H_

#include <string>
#include <utility>
#include <vector>

#include "mongo_msg_db_msgs/Find.h"
#include "rapid_pr2/joint_states.h"
#include "rapid_ros/service_client.h"
#include "rapidjson/document.h"
#include "robot_state_publisher/robot_state_publisher.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "rws_pr2_pbd/StopRobotState.h"
#include "tf/transform_broadcaster.h"

namespace pr2_pbd {
// StateServer publishes the robot state for PbD action steps.
// It reads joint positions from PbD actions (stored in MongoDB) and translates
// them into full robot states. A separate "world" is created for each client,
// although all the worlds are connected in a single TF tree to avoid excessive
// TF errors from other programs.
//
// Call Subscribe to subscribe a client to the robot state for an action step.
// Call Unsubscribe to unsubscribe the client.
//
// Call Publish to actually publish the robot states. The robot states will
// be published under the TF prefix: /pr2_pbd/{client_id}
class StateServer {
 public:
  StateServer(const robot_state_publisher::RobotStatePublisher& pub,
              rapid_ros::ServiceClientInterface<mongo_msg_db_msgs::Find>& find);

  // ROS Service handlers corresponding to Subscribe and Unsubscribe below.
  bool ServeSubscription(rws_pr2_pbd::PublishRobotState::Request& req,
                         rws_pr2_pbd::PublishRobotState::Response& res);
  bool ServeUnsubscription(rws_pr2_pbd::StopRobotState::Request& req,
                           rws_pr2_pbd::StopRobotState::Response& res);

  // Subscribes the given client to the robot state for a given action step.
  //
  // Args:
  //  client_id: A unique ID for each client, e.g., "0.12345"
  //  action_id: The MongoDB ObjectId of the PbD action to subscribe to, as a
  //    string. Actions are assumed to be stored in the "actions" collection of
  //    the "pr2_pbd" database.
  //  step_num: The step number of the action to subscribe to.
  //
  // Returns: true on success, false otherwise.
  bool Subscribe(const std::string& client_id, const std::string& action_id,
                 const int step_num);

  // Unsubscribes the given client from receiving updates.
  //
  // Args:
  //  client_id: The client to unsubscribe.
  void Unsubscribe(const std::string& client_id);

  // Publish robot state once. This should be called in a loop to continuously
  // publish TFs.
  void Publish();

 private:
  bool ParseJoints(const rapidjson::Value& step,
                   rapid::pr2::JointStates* joints);
  bool ParseArm(const rapidjson::Value& arm, const std::string& side_prefix,
                rapid::pr2::JointStates* joints);
  std::string tf_prefix(const std::string& client_id);
  robot_state_publisher::RobotStatePublisher pub_;
  std::map<std::string, rapid::pr2::JointStates>
      states_;  // Maps client IDs to robot states
  rapid_ros::ServiceClientInterface<mongo_msg_db_msgs::Find>& find_;
  tf::TransformBroadcaster tf_broadcaster_;
};
}  // namespace pr2_pbd

#endif  // _PR2_PBD_STATE_SERVER_H_
