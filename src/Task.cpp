#include "pr2_demo_lib/Task.h"

void executeAction(Pr2Robot* robot, const action_t& action)
{
  std::cout << "in executeAction: " << action.type << std::endl;
  if(action.type == "arm")
  {
    robot->startTrajectory(robot->createArmTrajectory(action.rarm_position));
    robot->waitForArmTrajectory(action.follow_hand, action.invert_hand, action.speed);
  }
  else if(action.type == "arms")
  {
    robot->startTrajectory(robot->createArmTrajectory(action.rarm_position, true, action.duration), true);
    robot->startTrajectory(robot->createArmTrajectory(action.larm_position, false, action.duration), false);
    robot->waitForArmsTrajectory();
  }
  else if(action.type == "gripper")
  {
    if(action.gripper == gripper_open)
      robot->openGripper();
    else
      robot->closeGripper();
  }
  else if(action.type == "head")
  {
    if(action.head == head_front)
      robot->lookFront();
  }
  else if(action.type == "sound")
    robot->speak(action.speak_sound, "");
  else if(action.type == "speak")
    robot->say(action.speak_str);
  else if(action.type == "delay")
    usleep(action.delay_ms * 1000);
  else if(action.type == "startChrono")
    robot->startChrono();
  else if(action.type == "stopChrono")
    robot->stopChrono();
  else if(action.type == "lookHand")
  {
    if(action.invert_hand)
      robot->lookInvertHand(true);
    else
      robot->lookHand(true);
  }
}

void executeTask(Pr2Robot* robot, const std::vector<action_t>& task)
{
  for(const auto& action : task)
    executeAction(robot, action);
}