#ifndef _PR2_PBD_STATE_SERVER_H_
#define _PR2_PBD_STATE_SERVER_H_

#include <string>
#include <utility>
#include <vector>

#include "kdl/tree.hpp"
#include "rapidjson/document.h"
#include "rapid_pr2/joint_states.h"
#include "robot_state_publisher/robot_state_publisher.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "urdf/model.h"

namespace pr2_pbd {
typedef std::pair<rapid::pr2::JointStates, std::string> RobotState;

// StateServer publishes the robot state for PbD action steps.
// It reads joint positions from PbD actions (stored in MongoDB) and translates
// them into full robot states, which are each published to their own TF prefix.
//
// Call AddAction for all actions you want the state for. There will be
// separate
// robot states published for each step of the action.
//
// Call Publish to actually publish the robot states. The robot states will
// be
// published under the TF prefix: /pr2_pbd/{action_id}/{step_num}
// E.g., /pr2_pbd/abcdefg/0 for the first step of the action with ID
// abcdefg.
class StateServer {
 public:
  StateServer(const ros::NodeHandle& nh);
  ~StateServer();

  bool ServeAdd(rws_pr2_pbd::PublishRobotState::Request& req,
                rws_pr2_pbd::PublishRobotState::Response& res);

  // Add an action to publish, given a MongoDB ObjectId as a string. Actions are
  // assumed to be stored in the "actions" collection of the "pr2_pbd" database.
  bool AddAction(const std::string& action_id);

  // Publish robot state once. This should be called in a loop to continuously
  // publish TFs.
  void Publish();

 private:
  bool ParseJoints(const rapidjson::Value& step,
                   rapid::pr2::JointStates* joints);
  bool ParseArm(const rapidjson::Value& arm, const std::string& side_prefix,
                rapid::pr2::JointStates* joints);
  ros::NodeHandle nh_;
  urdf::Model model_;
  KDL::Tree tree_;
  robot_state_publisher::RobotStatePublisher* pub_;
  std::map<std::string, std::vector<RobotState> >
      states_;  // Maps action IDs to states to publish
  ros::ServiceClient find_;
};
}  // namespace pr2_pbd

#endif  // _PR2_PBD_STATE_SERVER_H_
