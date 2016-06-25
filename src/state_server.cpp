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
#include "rws_pr2_pbd/StopRobotState.h"

using mongo_msg_db_msgs::Find;
using mongo_msg_db_msgs::FindRequest;
using mongo_msg_db_msgs::FindResponse;
using rapid::pr2::JointStates;
using rapid_ros::ServiceClientInterface;
using robot_state_publisher::RobotStatePublisher;
using rws_pr2_pbd::PublishRobotState;
using rws_pr2_pbd::StopRobotState;
using std::string;
using std::map;
using rapidjson::Value;

namespace pr2_pbd {
StateServer::StateServer(const RobotStatePublisher& pub,
                         ServiceClientInterface<Find>& find)
    : pub_(pub), states_(), find_(find), tf_broadcaster_() {}

bool StateServer::ServeSubscription(PublishRobotState::Request& req,
                                    PublishRobotState::Response& res) {
  bool success = Subscribe(req.client_id, req.action_id, req.step_num);
  if (!success) {
    res.error = "Failed to subscribe client " + req.client_id +
                " to robot state for action " + req.action_id;
  } else {
    res.tf_prefix = tf_prefix(req.client_id);
  }
  return true;
}

bool StateServer::ServeUnsubscription(StopRobotState::Request& req,
                                      StopRobotState::Response& res) {
  Unsubscribe(req.client_id);
  return true;
}

bool StateServer::Subscribe(const string& client_id, const string& action_id,
                            int step_num) {
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
  const Value& seq = doc["sequence"]["seq"];
  if (step_num >= static_cast<int>(seq.Size())) {
    ROS_ERROR("Failed to parse action step: step # does not exist");
    return false;
  }
  const Value& step = seq[step_num];
  JointStates joints;
  if (!ParseJoints(step, &joints)) {
    return false;
  }
  string tf_prefix = "/pr2_pbd/" + client_id;
  ROS_INFO("Subscribed client %s to action %s, step %d", client_id.c_str(),
           action_id.c_str(), step_num);
  states_[client_id] = joints;

  return true;
}

void StateServer::Unsubscribe(const string& client_id) {
  size_t removed_count = states_.erase(client_id);
  if (removed_count == 0) {
    ROS_WARN("%s is already not subscribed", client_id.c_str());
  } else {
    ROS_INFO("Unsubscribed client %s", client_id.c_str());
  }
}

void StateServer::Publish() {
  tf::Transform identity;
  identity.setIdentity();
  for (std::map<string, JointStates>::const_iterator it = states_.begin();
       it != states_.end(); ++it) {
    const string& client_id = it->first;
    const JointStates& joints = it->second;
    const string prefix = tf_prefix(client_id);
    pub_.publishFixedTransforms(prefix);
    pub_.publishTransforms(joints.joint_positions(), ros::Time::now(), prefix);
    const string client_base_link = prefix + "/base_footprint";
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        identity, ros::Time::now(), "base_footprint", client_base_link));
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

string StateServer::tf_prefix(const std::string& client_id) {
  return "/pr2_pbd/" + client_id;
}
}  // namespace pr2_pbd
