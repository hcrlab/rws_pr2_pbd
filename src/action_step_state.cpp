#include "rws_pr2_pbd/action_step_state.h"

#include <map>
#include <string>
#include <vector>

#include "pviz/pviz.h"
#include "rapid_pr2/joint_states.h"

namespace pr2_pbd {
ActionStepState::ActionStepState() : joint_states_() {}

rapid::pr2::JointStates ActionStepState::joint_states() const {
  return joint_states_;
}

void ActionStepState::set_joint_states(rapid::pr2::JointStates joint_states) {
  joint_states_ = joint_states;
}
}  // namespace pr2_pbd
