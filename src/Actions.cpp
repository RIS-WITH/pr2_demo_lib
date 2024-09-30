#include "pr2_demo_lib/Actions.h"

action_t createAction(const std::vector<std::vector<double>>& rarm_position, bool follow_hand, bool invert_hand, double speed)
{
  action_t action;
  action.type = "arm";
  action.rarm_position = rarm_position;
  action.follow_hand = follow_hand;
  action.invert_hand = invert_hand;
  action.single_double = speed;
  return action;
}

action_t createAction(const std::vector<std::vector<double>>& larm_position, const std::vector<std::vector<double>>& rarm_position, double duration)
{
  action_t action;
  action.type = "arms";
  action.larm_position = larm_position;
  action.rarm_position = rarm_position;
  action.single_double = duration;
  return action;
}

action_t createAction(GripperState_e gripper)
{
  action_t action;
  action.type = "gripper";
  action.gripper = gripper;
  return action;
}

action_t createAction(HeadState_e head)
{
  action_t action;
  action.type = "head";
  action.head = head;
  return action;
}

action_t createActionTorso(double pose)
{
  action_t action;
  action.type = "torso";
  action.single_double = pose;
  return action;
}

action_t createActionMoveFront(double dist)
{
  action_t action;
  action.type = "moveFront";
  action.single_double = dist;
  return action;
}

action_t createActionMoveRight(double dist)
{
  action_t action;
  action.type = "moveRight";
  action.single_double = dist;
  return action;
}

action_t createActionMove(double dist_x, double dist_y)
{
  action_t action;
  action.type = "move";
  action.single_double = dist_x;
  action.second_double = dist_y;
  return action;
}

action_t createActionTurn(double angle)
{
  action_t action;
  action.type = "turn";
  action.single_double = angle;
  return action;
}

action_t createActionLookHand(bool invert)
{
  action_t action;
  action.type = "lookHand";
  action.invert_hand = invert;
  return action;
}

action_t createAction(int speak_sound)
{
  action_t action;
  action.type = "sound";
  action.single_int = speak_sound;
  return action;
}

action_t createActionSpeak(const std::string& msg)
{
  action_t action;
  action.type = "speak";
  action.speak_str = msg;
  return action;
}

action_t createActionDelay(int delay_ms)
{
  action_t action;
  action.type = "delay";
  action.single_int = delay_ms;
  return action;
}

action_t createActionStartChrono()
{
  action_t action;
  action.type = "startChrono";
  return action;
}

action_t createActionStopChrono()
{
  action_t action;
  action.type = "stopChrono";
  return action;
}

action_t createActionSynchro(const std::string& ip_addr)
{
  action_t action;
  action.type = "synchro";
  action.ip_ws = ip_addr;
  return action;
}

action_t createActionLaunchSynchro(const std::string& ip_addr)
{
  action_t action;
  action.type = "launchSynchro";
  action.ip_ws = ip_addr;
  return action;
}

action_t createActionWaitSynchro(const std::string& ip_addr)
{
  action_t action;
  action.type = "waitSynchro";
  action.ip_ws = ip_addr;
  return action;
}
