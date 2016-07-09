#include "rws_pr2_pbd/state_server.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rapid_pr2/joint_states.h"
#include "rapidjson/document.h"
#include "robot_state_publisher/robot_state_publisher.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"

using mongo_msg_db_msgs::Find;
using mongo_msg_db_msgs::FindRequest;
using mongo_msg_db_msgs::FindResponse;
using rapid::pr2::JointStates;
using rapid_ros::ServiceClientInterface;
using robot_state_publisher::RobotStatePublisher;
using rws_pr2_pbd::PublishRobotState;
using std::string;
using std::map;
using rapidjson::Value;

namespace pr2_pbd {
const char* StateServer::kDb = "pr2_pbd";
const char* StateServer::kCollection = "actions";

StateServer::StateServer(const RobotStatePublisher& pub,
                         ServiceClientInterface<Find>& find)
    : pub_(pub), states_(), find_(find), tf_broadcaster_() {}

bool StateServer::ServeSubscription(PublishRobotState::Request& req,
                                    PublishRobotState::Response& res) {
  ActionStepKey key(req.action_id, req.step_num);
  bool success = Subscribe(key);
  if (!success) {
    std::stringstream ss;
    ss << "Failed to publish robot state for action " << req.action_id
       << " step " << req.step_num;
    res.error = ss.str();
  } else {
    res.tf_prefix = tf_prefix(key);
  }
  return true;
}

bool StateServer::Subscribe(const ActionStepKey& key) {
  if (states_.find(key) != states_.end()) {
    return true;
  }
  FindRequest req;
  req.collection.db = kDb;
  req.collection.collection = kCollection;
  const string& action_id = key.first;
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
  const Value& seq = doc["sequence"]["seq"];
  for (int step_num = 0; step_num < static_cast<int>(seq.Size()); ++step_num) {
    const Value& step = seq[step_num];
    ActionStepKey step_key(key.first, step_num);
    JointStates joints;
    if (!ParseJoints(step, &joints)) {
      return false;
    }
    states_[step_key] = joints;
    ROS_INFO("Will publish action %s, step %d", action_id.c_str(), step_num);
  }
  ROS_INFO("Publishing %ld steps total", states_.size());

  return true;
}

void StateServer::Publish() {
  tf::Transform identity;
  identity.setIdentity();
  for (std::map<ActionStepKey, JointStates>::const_iterator it =
           states_.begin();
       it != states_.end(); ++it) {
    const ActionStepKey& key = it->first;
    const JointStates& joints = it->second;
    const string prefix = tf_prefix(key);
    pub_.publishFixedTransforms(prefix);
    pub_.publishTransforms(joints.joint_positions(), ros::Time::now(), prefix);
    const string step_base_link = prefix + "/base_footprint";
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        identity, ros::Time::now(), "/base_footprint", step_base_link));
  }
}

bool StateServer::ParseJoints(const Value& step, JointStates* joints) {
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

bool StateServer::ParseArm(const Value& arm, const string& side_prefix,
                           rapid::pr2::JointStates* joints) {
  if (!arm.HasMember("joint_pose")) {
    ROS_ERROR("Failed to parse action step: no joint_pose field");
    return false;
  }
  const Value& arm_joints = arm["joint_pose"];
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

  std::map<string, double> updates;
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

string StateServer::tf_prefix(const ActionStepKey& key) {
  std::stringstream ss;
  const std::string& action_id = key.first;
  const int step_num = key.second;
  ss << "/pr2_pbd/" << action_id << "/" << step_num;
  return ss.str();
}
}  // namespace pr2_pbd
