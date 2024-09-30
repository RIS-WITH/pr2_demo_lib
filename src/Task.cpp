#include "pr2_demo_lib/Task.h"

void executeAction(Pr2Robot* robot, const action_t& action)
{
  std::cout << "in executeAction: " << action.type << std::endl;
  if(action.type == "arm")
  {
    robot->startTrajectory(robot->createArmTrajectory(action.rarm_position));
    robot->waitForArmTrajectory(action.follow_hand, action.invert_hand, action.single_double);
  }
  else if(action.type == "arms")
  {
    robot->startTrajectory(robot->createArmTrajectory(action.rarm_position, true, action.single_double), true);
    robot->startTrajectory(robot->createArmTrajectory(action.larm_position, false, action.single_double), false);
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
  else if(action.type == "torso")
    robot->moveTorso(action.single_double);
  else if(action.type == "moveFront")
    robot->moveFront(action.single_double);
  else if(action.type == "moveRight")
    robot->moveRight(action.single_double);
  else if(action.type == "move")
    robot->move(action.single_double, action.second_double);
  else if(action.type == "sound")
    robot->speak(action.single_int, "");
  else if(action.type == "speak")
    robot->say(action.speak_str);
  else if(action.type == "delay")
    usleep(action.single_int * 1000);
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
  else if(action.type == "launchSynchro")
    robot->launchSynchro(action.ip_ws);
  else if(action.type == "waitSynchro")
    robot->waitSynchro(action.ip_ws);
  else if(action.type == "synchro")
    robot->synchro(action.ip_ws);
}

void executeTask(Pr2Robot* robot, const std::vector<action_t>& task)
{
  for(const auto& action : task)
    executeAction(robot, action);
}
