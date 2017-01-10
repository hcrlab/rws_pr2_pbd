#ifndef _PR2_PBD_ACTION_STEP_STATE_H_
#define _PR2_PBD_ACTION_STEP_STATE_H_

#include "pviz/pviz.h"
#include "rapid_pr2/joint_states.h"

// Represents the state of a single PbD action step.
// TODO(jstn): This should be a core part of pr2_pbd to begin with. Making our
// own version for now.

namespace pr2_pbd {
class ActionStepState {
 public:
  ActionStepState();

  rapid::pr2::JointStates joint_states() const;
  void set_joint_states(rapid::pr2::JointStates joint_states);

 private:
  rapid::pr2::JointStates joint_states_;
};
}  // namespace pr2_pbd

#endif  // _PR2_PBD_ACTION_STEP_STATE_H_
