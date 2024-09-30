#ifndef PR2_DEMO_LIB_PR2ROBOT_H
#define PR2_DEMO_LIB_PR2ROBOT_H

#include "pr2_demo_lib/Types.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <navigation_position_refinement/BlindMovementAction.h>

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/sound_play.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_listener.h"

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<navigation_position_refinement::BlindMovementAction> NavClient;

class Pr2Robot
{
protected:
  TrajClient *rarm_client_;
  TrajClient *larm_client_;
  TorsoClient *torso_client_;
  GripperClient *gripper_client_;
  NavClient *nav_client_;

  ros::ServiceServer synchro_srv_;
  std::atomic_bool free_;

  PointHeadClient *point_head_client_;

  ros::NodeHandle n_;
  ros::Publisher sound_pub_;
  ros::Publisher tts_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string result_;
  std::chrono::time_point<std::chrono::steady_clock> time_start_;
  std::chrono::time_point<std::chrono::steady_clock> time_stop_;

  std::condition_variable condition_variable_;

  bool is_real_;
  std::string lang_;
  std::map<std::string, std::string> en2fr_;

public:
  Pr2Robot(bool real = true);
  ~Pr2Robot();

  void initPose();

  void say(const std::string &txt);
  void speak(int sound, const std::string &sentence);

  void openGripper() { moveGripper(gripper_open); }
  void closeGripper() { moveGripper(gripper_close); }

  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool right = true);
  pr2_controllers_msgs::JointTrajectoryGoal createArmTrajectory(const std::vector<std::vector<double>> &positions, bool right = true, double duration = 1.1);

  void setLang(const std::string &lang);

  void lookFront();
  void lookHand(bool wait = false, double speed = 0.0);
  void lookInvertHand(bool wait = false, double speed = 0.0);

  void waitForArmTrajectory(bool follow_hand = false, bool invert_hand = false, double speed = 0);
  void waitForArmsTrajectory();

  void moveTorso(double pose, bool wait = false, double speed = 0.0);

  void moveFront(double dist);
  void moveRight(double dist);

  void startChrono() { time_start_ = std::chrono::steady_clock::now(); }
  void stopChrono() { time_stop_ = std::chrono::steady_clock::now(); }
  std::chrono::duration<double> getChrono() { return time_stop_ - time_start_; }

  void synchro(const std::string& ip_addr);
  void launchSynchro(const std::string& ip_addr);
  void waitSynchro(const std::string& ip_addr);

private:
  void moveGripper(GripperState_e state);
  void lookAt(geometry_msgs::PointStamped point, bool wait_for = false, double speed = 0.0);
  bool callback_wait_service(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
};

#endif // PR2_DEMO_LIB_PR2ROBOT_H
