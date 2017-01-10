#ifndef _PR2_PBD_STATE_SERVER_H_
#define _PR2_PBD_STATE_SERVER_H_

#include <string>
#include <utility>
#include <vector>

#include "mongo_msg_db_msgs/Find.h"
#include "pviz/pviz.h"
#include "rapid_pr2/joint_states.h"
#include "rapid_ros/service_client.h"
#include "rapidjson/document.h"
#include "ros/ros.h"

#include "rws_pr2_pbd/action_step_state.h"
#include "rws_pr2_pbd/PublishRobotState.h"

namespace pr2_pbd {

typedef std::pair<std::string, int> ActionStepKey;

// StateServer publishes the robot state for PbD action steps.
// It reads joint positions from PbD actions (stored in MongoDB) and translates
// them into full robot states.
//
// Each action step has its own TF prefix under which all robot states and
// markers are published. Due to issues with MoveIt!
// (https://github.com/hcrlab/rws_pr2_pbd/issues/4), once a state is published
// we never stop publishing it. For the same reason, it's important to ensure
// that this server lives at least as long as MoveIt does.
//
// Call Subscribe to start publishing the robot state for an action step.
//
// Call Publish to actually publish the robot states. The robot states will
// be published under the TF prefix: /pr2_pbd/{action_id}/{step_num}
class StateServer {
 public:
  StateServer(rapid_ros::ServiceClientInterface<mongo_msg_db_msgs::Find>& find);

  // ROS Service handler corresponding to Subscribe below.
  bool ServeSubscription(rws_pr2_pbd::PublishRobotState::Request& req,
                         rws_pr2_pbd::PublishRobotState::Response& res);

  // Subscribes the given client to the robot state for a given action step.
  //
  // Args:
  //  action_id: The MongoDB ObjectId of the PbD action to subscribe to, as a
  //    string. Actions are assumed to be stored in the "actions" collection of
  //    the "pr2_pbd" database.
  //  step_num: The step number of the action to subscribe to.
  //
  // Returns: true on success, false otherwise.
  bool Subscribe(const ActionStepKey& key);

  // The name of the MongoDB database containing the actions.
  static const char* kDb;
  // The name of the MongoDB collection containing the actions.
  static const char* kCollection;
  // Assumed torso height (major TODO: save torso height as part of action
  // step).
  static const double kTorsoHeight;
  static const double kDefaultHue;
  static const double kDefaultId;

 private:
  bool ParseJoints(const rapidjson::Value& step,
                   rapid::pr2::JointStates* joints);
  bool ParseArm(const rapidjson::Value& arm, const std::string& side_prefix,
                rapid::pr2::JointStates* joints);
  void Publish(const ActionStepKey& key, const ActionStepState& state);
  std::string tf_prefix(const ActionStepKey& key);
  ros::NodeHandle nh_;
  std::map<ActionStepKey, ActionStepState>
      states_;  // Maps action steps to robot states
  std::map<ActionStepKey, ros::Publisher>
      publishers_;  // Maps action steps to publishers
  rapid_ros::ServiceClientInterface<mongo_msg_db_msgs::Find>& find_;
  PViz pviz_;
};
}  // namespace pr2_pbd

#endif  // _PR2_PBD_STATE_SERVER_H_
