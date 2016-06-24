#include "rws_pr2_pbd/state_server.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "mongo_msg_db_msgs/Find.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rapid_pr2/joint_states.h"
#include "rapidjson/document.h"
#include "robot_state_publisher/robot_state_publisher.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "urdf/model.h"

using mongo_msg_db_msgs::FindRequest;
using mongo_msg_db_msgs::FindResponse;
using rapid::pr2::JointStates;

namespace pr2_pbd {
StateServer::StateServer(const ros::NodeHandle& nh)
    : nh_(nh),
      model_(),
      tree_(),
      pub_(),
      states_(),
      find_(nh_.serviceClient<mongo_msg_db_msgs::Find>("mongo_msg_db/find")) {
  model_.initParam("robot_description");
  kdl_parser::treeFromUrdfModel(model_, tree_);
  pub_ = new robot_state_publisher::RobotStatePublisher(tree_);
}

StateServer::~StateServer() {
  if (pub_ != NULL) {
    delete pub_;
  }
}

bool StateServer::ServeAdd(rws_pr2_pbd::PublishRobotState::Request& req,
                           rws_pr2_pbd::PublishRobotState::Response& res) {
  bool success = AddAction(req.action_id);
  if (!success) {
    res.error = "Failed to publish robot state for action " + req.action_id;
  }
  return true;
}

bool StateServer::AddAction(const std::string& action_id) {
  if (states_.find(action_id) != states_.end()) {
    return true;
  }
  FindRequest req;
  req.collection.db = "pr2_pbd";
  req.collection.collection = "actions";
  req.id = action_id;
  FindResponse response;
  bool success = find_.call(req, response);
  if (!success) {
    ROS_ERROR("Failed to call Find in robot state server.");
    return false;
  }

  rapidjson::Document doc;
  doc.Parse(response.message.json.c_str());
  if (!doc.HasMember("sequence") || !doc["sequence"].HasMember("seq")) {
    ROS_ERROR("Failed to parse action step: no sequence or seq field");
    return false;
  }
  const rapidjson::Value& seq = doc["sequence"]["seq"];
  std::vector<RobotState> states;
  for (size_t i = 0; i < seq.Size(); ++i) {
    JointStates joints;
    const rapidjson::Value& step = seq[i];
    if (!ParseJoints(step, &joints)) {
      return false;
    }
    std::stringstream tf_prefix;
    tf_prefix << "/pr2_pbd/" << action_id << "/" << i;
    ROS_INFO("Publishing robot state to TF prefix: %s",
             tf_prefix.str().c_str());
    RobotState state(joints, tf_prefix.str());
    states.push_back(state);
  }
  states_[action_id] = states;

  return true;
}

void StateServer::Publish() {
  for (size_t i = 0; i < states_.size(); ++i) {
    const RobotState& state = states_[i];
    pub_->publishFixedTransforms(state.second);
    pub_->publishTransforms(state.first.joint_positions(), ros::Time::now(),
                            state.second);
  }
}

bool StateServer::ParseJoints(const rapidjson::Value& step,
                              JointStates* joints) {
  if (!step.HasMember("armTarget")) {
    ROS_ERROR("Failed to parse action step: no armTarget field");
    return false;
  }
  if (!step["armTarget"].HasMember("lArm")) {
    ROS_ERROR("Failed to parse action step: no lArm field");
    return false;
  }
  if (!ParseArm(step["armTarget"]["lArm"], "l", joints)) {
    return false;
  }
  if (!step["armTarget"].HasMember("rArm")) {
    ROS_ERROR("Failed to parse action step: no rArm field");
    return false;
  }
  if (!ParseArm(step["armTarget"]["rArm"], "r", joints)) {
    return false;
  }
  return true;
}

bool StateServer::ParseArm(const rapidjson::Value& arm,
                           const std::string& side_prefix,
                           rapid::pr2::JointStates* joints) {
  if (!arm.HasMember("joint_pose")) {
    ROS_ERROR("Failed to parse action step: no joint_pose field");
    return false;
  }
  const rapidjson::Value& arm_joints = arm["joint_pose"];
  if (arm_joints.Size() != 7) {
    ROS_ERROR("Failed to parse action step: # joints != 7");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    if (!arm_joints[i].IsDouble()) {
      ROS_ERROR("Failed to parse action step: joint value was not a double");
      return false;
    }
  }

  std::map<std::string, double> updates;
  updates[side_prefix + "_shoulder_pan_joint"] = arm_joints[0].GetDouble();
  updates[side_prefix + "_shoulder_lift_joint"] = arm_joints[1].GetDouble();
  updates[side_prefix + "_upper_arm_roll_joint"] = arm_joints[2].GetDouble();
  updates[side_prefix + "_elbow_flex_joint"] = arm_joints[3].GetDouble();
  updates[side_prefix + "_forearm_roll_joint"] = arm_joints[4].GetDouble();
  updates[side_prefix + "_wrist_flex_joint"] = arm_joints[5].GetDouble();
  updates[side_prefix + "_wrist_roll_joint"] = arm_joints[6].GetDouble();
  joints->Set(updates);
  return true;
}
}  // namespace pr2_pbd
