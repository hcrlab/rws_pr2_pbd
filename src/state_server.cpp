#include "rws_pr2_pbd/state_server.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "pviz/pviz.h"
#include "rapid_pr2/joint_states.h"
#include "rapidjson/document.h"
#include "ros/ros.h"
#include "rws_pr2_pbd/PublishRobotState.h"
#include "visualization_msgs/MarkerArray.h"

using mongo_msg_db_msgs::Find;
using mongo_msg_db_msgs::FindRequest;
using mongo_msg_db_msgs::FindResponse;
using rapid::pr2::JointStates;
using rapid_ros::ServiceClientInterface;
using rapidjson::Value;
using rws_pr2_pbd::PublishRobotState;
using std::map;
using std::string;
using std::vector;

namespace pr2_pbd {
const char* StateServer::kDb = "pr2_pbd";
const char* StateServer::kCollection = "actions";
const double StateServer::kTorsoHeight = 0.15;
const double StateServer::kDefaultHue = 0;
const double StateServer::kDefaultId = 0;

StateServer::StateServer(ServiceClientInterface<Find>& find)
    : nh_(), states_(), find_(find), pviz_("", "/base_link") {}

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
    res.prefix = tf_prefix(key);
  }
  return true;
}

bool StateServer::Subscribe(const ActionStepKey& key) {
  if (states_.find(key) != states_.end()) {
    Publish(key, states_[key]);
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
    ActionStepState state;
    state.set_joint_states(joints);
    states_[step_key] = state;
    publishers_[step_key] = nh_.advertise<visualization_msgs::MarkerArray>(
        tf_prefix(step_key) + "/robot", 100, true);
    ROS_INFO("Cached action %s, step %d", action_id.c_str(), step_num);
  }
  ROS_INFO("Cached %ld steps total", states_.size());
  Publish(key, states_[key]);

  return true;
}

void StateServer::Publish(const ActionStepKey& key,
                          const ActionStepState& state) {
  ROS_INFO("Visualizing action %s, step %d", key.first.c_str(), key.second);
  std::vector<double> r_joints(7, 0);
  std::vector<double> l_joints(7, 0);
  std::vector<double> base_pos(3, 0);
  const std::map<std::string, double>& joint_positions =
      state.joint_states().joint_positions();
  r_joints[0] = joint_positions.at("r_shoulder_pan_joint");
  r_joints[1] = joint_positions.at("r_shoulder_lift_joint");
  r_joints[2] = joint_positions.at("r_upper_arm_roll_joint");
  r_joints[3] = joint_positions.at("r_elbow_flex_joint");
  r_joints[4] = joint_positions.at("r_forearm_roll_joint");
  r_joints[5] = joint_positions.at("r_wrist_flex_joint");
  r_joints[6] = joint_positions.at("r_wrist_roll_joint");
  l_joints[0] = joint_positions.at("l_shoulder_pan_joint");
  l_joints[1] = joint_positions.at("l_shoulder_lift_joint");
  l_joints[2] = joint_positions.at("l_upper_arm_roll_joint");
  l_joints[3] = joint_positions.at("l_elbow_flex_joint");
  l_joints[4] = joint_positions.at("l_forearm_roll_joint");
  l_joints[5] = joint_positions.at("l_wrist_flex_joint");
  l_joints[6] = joint_positions.at("l_wrist_roll_joint");

  std::vector<geometry_msgs::PoseStamped> poses;
  if (!pviz_.computeFKforVisualizationWithKDL(r_joints, l_joints, base_pos,
                                              kTorsoHeight, poses)) {
    ROS_WARN("Unable to compute forward kinematics.");
  } else {
    visualization_msgs::MarkerArray marker_array =
        pviz_.getRobotMeshesMarkerMsg(kDefaultHue, tf_prefix(key), kDefaultId,
                                      poses, true);
    publishers_[key].publish(marker_array);
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
  ss << "pr2_pbd/" << action_id << "/" << step_num;
  return ss.str();
}
}  // namespace pr2_pbd
