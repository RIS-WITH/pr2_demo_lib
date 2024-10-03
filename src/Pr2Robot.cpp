#include "pr2_demo_lib/Pr2Robot.h"

#include "rosbridge_ws_client.hpp"

Pr2Robot::Pr2Robot(bool real) : tf_listener_(tf_buffer_), free_(false)
{
  is_real_ = real;

  rarm_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
  if(is_real_)
    while(!rarm_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the right arm joint_trajectory_action server");

  larm_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
  if(is_real_)
    while(!larm_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the left arm joint_trajectory_action server");

  gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
  if(is_real_)
    while(!gripper_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");

  nav_client_ = new NavClient("blind_movement", true);
  // if(is_real_)
  //   while(!nav_client_->waitForServer(ros::Duration(1.0)) && ros::ok())
  //     ROS_INFO("Waiting for the blind_movement server to come up");

  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  if(is_real_)
    while(!point_head_client_->waitForServer(ros::Duration(1.0)) && ros::ok())
      ROS_INFO("Waiting for the point_head_action server to come up");

  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
  if(is_real_)
    while(!torso_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
      ROS_INFO("Waiting for the torso action server to come up");

  sound_pub_ = n_.advertise<sound_play::SoundRequest>("robotsound", 1000);
  tts_pub_ = n_.advertise<std_msgs::String>("say", 1000);

  synchro_srv_ = n_.advertiseService("/synchro_action", &Pr2Robot::callback_wait_service, this);

  lang_ = "fr";
  std::cout << "Pr2Robot is ready" << std::endl;
}

Pr2Robot::~Pr2Robot()
{
  delete rarm_client_;
  delete larm_client_;

  delete gripper_client_;

  delete nav_client_;

  delete point_head_client_;
}

void Pr2Robot::initPose()
{
  closeGripper();
  moveTorso(0);
  startTrajectory(createArmTrajectory({
                                        {0.096, 0.802, 0.198, -2.049, -0.020, -1.877, 31.447}
  },
                                      true, 2),
                  true);
  startTrajectory(createArmTrajectory({
                                        {0.096, 0.802, 0.198, -2.049, -0.020, -1.877, 31.447}
  },
                                      false, 2),
                  false);
  waitForArmsTrajectory();
}

void Pr2Robot::say(const std::string& txt)
{
  std_msgs::String msg;
  if(lang_ == "en")
    msg.data = txt;
  else if(en2fr_.find(txt) != en2fr_.end())
    msg.data = en2fr_[txt];
  else
    msg.data = txt;
  tts_pub_.publish(msg);
  // usleep(120000*txt.size());
}

void Pr2Robot::speak(int sound, const std::string& sentence)
{
  sound_play::SoundRequest sound_request;

  sound_request.sound = sound; // 1-5 for builtin sounds -3 for sentence
  sound_request.command = 1;
  sound_request.volume = 10.0;
  sound_request.arg = sentence;
  sound_request.arg2 = "voice_kal_diphone";

  sound_pub_.publish(sound_request);
  // usleep(120000*sentence.size());
}

void Pr2Robot::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool right)
{
  if(!is_real_)
    return;

  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  if(right)
    rarm_client_->sendGoal(goal);
  else
    larm_client_->sendGoal(goal);
}

pr2_controllers_msgs::JointTrajectoryGoal Pr2Robot::createArmTrajectory(const std::vector<std::vector<double>>& positions, bool right, double duration)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_elbow_flex_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_forearm_roll_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_wrist_flex_joint");
  goal.trajectory.joint_names.push_back(std::string(right ? "r" : "l") + "_wrist_roll_joint");

  goal.trajectory.points.resize(positions.size());

  for(size_t i = 0; i < positions.size(); i++)
  {
    goal.trajectory.points[i].positions = positions[i];
    goal.trajectory.points[i].velocities = std::vector<double>(7, 0);
    goal.trajectory.points[i].time_from_start = ros::Duration((i + 1) * duration);
  }

  return goal;
}

void Pr2Robot::setLang(const std::string& lang)
{
  lang_ = lang;
}

void Pr2Robot::lookFront()
{
  if(!is_real_)
    return;

  geometry_msgs::PointStamped point;
  point.header.frame_id = "base_link";
  point.point.x = 6;
  point.point.y = 0;
  point.point.z = 0;
  lookAt(point, true);
}

void Pr2Robot::lookHand(bool wait, double speed)
{
  if(!is_real_)
    return;

  geometry_msgs::PointStamped point;
  point.header.frame_id = "r_gripper_tool_frame";
  point.point.x = 0;
  point.point.y = 0;
  point.point.z = 0;
  lookAt(point, wait, speed);
}

void Pr2Robot::lookInvertHand(bool wait, double speed)
{
  if(!is_real_)
    return;

  geometry_msgs::PointStamped point;
  point.header.frame_id = "r_gripper_tool_frame";
  point.point.x = 0;
  point.point.y = 0;
  point.point.z = 0;

  try
  {
    geometry_msgs::PointStamped transformed_point;
    tf_buffer_.transform(point, transformed_point, "base_link");
    transformed_point.point.y = -transformed_point.point.y;
    lookAt(transformed_point, wait, speed);
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("Failure %s\n", ex.what());
  }
}

void Pr2Robot::waitForArmTrajectory(bool follow_hand, bool invert_hand, double speed)
{
  if(!is_real_)
    return;

  while(!rarm_client_->getState().isDone() && ros::ok())
  {
    if(follow_hand)
    {
      if(invert_hand == false)
        lookHand(false, speed);
      else
        lookInvertHand(false, speed);
    }

    usleep(50000);
  }
}

void Pr2Robot::waitForArmsTrajectory()
{
  if(!is_real_)
    return;

  while(!rarm_client_->getState().isDone() && !larm_client_->getState().isDone() && ros::ok())
    usleep(50000);
}

void Pr2Robot::moveTorso(double pose, bool wait, double speed)
{
  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = pose;
  goal.min_duration = ros::Duration(2.0);
  goal.max_velocity = (speed != 0) ? speed : 1.0;

  if(wait == false)
    torso_client_->sendGoal(goal);
  else
    torso_client_->sendGoalAndWait(goal, ros::Duration(10));
}

void Pr2Robot::moveGripper(GripperState_e state)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_goal;
  gripper_goal.command.position = state == gripper_open ? 0.08 : 0.0;
  gripper_goal.command.max_effort = state == gripper_open ? -1.0 : 50.0;

  ROS_INFO("Sending gripper goal");
  gripper_client_->sendGoal(gripper_goal);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper operated!");
  else
    ROS_INFO("The gripper failed.");
}

void Pr2Robot::moveFront(double dist)
{
  navigation_position_refinement::BlindMovementGoal goal;
  goal.x_movement = dist;
  goal.y_movement = 0;
  goal.linear_velocity = 0.1;
  goal.angular_velocity = 0.2;

  nav_client_->sendGoal(goal);
  nav_client_->waitForResult();
  if(nav_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The navigation operated!");
  else
    ROS_INFO("The navigation failed.");
}

void Pr2Robot::moveRight(double dist)
{
  navigation_position_refinement::BlindMovementGoal goal;
  goal.y_movement = -dist;
  goal.x_movement = 0;
  goal.linear_velocity = 0.1;
  goal.angular_velocity = 0.2;

  nav_client_->sendGoal(goal);
  nav_client_->waitForResult();
  if(nav_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The navigation operated!");
  else
    ROS_INFO("The navigation failed.");
}

void Pr2Robot::move(double dist_x, double dist_y)
{
  navigation_position_refinement::BlindMovementGoal goal;
  goal.y_movement = -dist_y;
  goal.x_movement = dist_x;
  goal.linear_velocity = 0.1;
  goal.angular_velocity = 0.2;

  nav_client_->sendGoal(goal);
  nav_client_->waitForResult();
  if(nav_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The navigation operated!");
  else
    ROS_INFO("The navigation failed.");
}

void Pr2Robot::turn(double angle)
{
  navigation_position_refinement::BlindMovementGoal goal;
  goal.theta_rotation = angle * M_PI / 180.;
  goal.angular_velocity = 0.2;
  goal.linear_velocity = 0.; // 0.1;

  nav_client_->sendGoal(goal);
  nav_client_->waitForResult();
  if(nav_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The navigation operated!");
  else
    ROS_INFO("The navigation failed.");
}

void Pr2Robot::lookAt(geometry_msgs::PointStamped point, bool wait_for, double speed)
{
  pr2_controllers_msgs::PointHeadGoal goal;
  goal.target = point;
  goal.pointing_frame = "narrow_stereo_r_stereo_camera_optical_frame";
  goal.pointing_axis.x = 0;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 1;

  goal.min_duration = ros::Duration(0.00001);

  if(wait_for)
    goal.max_velocity = (speed != 0) ? speed : 1.0;
  else
    goal.max_velocity = (speed != 0) ? speed : 0.5;

  if(wait_for == false)
    point_head_client_->sendGoal(goal);
  else
    point_head_client_->sendGoalAndWait(goal, ros::Duration(2));
}

void Pr2Robot::synchro(const std::string& ip_addr)
{
  if(ip_addr.empty() == false)
  {
    launchSynchro(ip_addr);
    waitSynchro(ip_addr);
  }
}

void Pr2Robot::launchSynchro(const std::string& ip_addr)
{
  if(ip_addr.empty() == false)
  {
    RosbridgeWsClient rbc(ip_addr);
    std::cout << " in Launch Synchro " << ip_addr << std::endl;
    rbc.addClient("service_advertiser");
    rbc.callService("/synchro_action", {}, {});
    std::cout << " Synchro done " << ip_addr << std::endl;
  }
}

bool Pr2Robot::callback_wait_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  std::cout << "callback_wait_service" << std::endl;
  free_ = true;
  return true;
}

void Pr2Robot::waitSynchro(const std::string& ip_addr)
{
  if(ip_addr.empty() == false)
  {
    while(!free_)
    {
      ROS_INFO("wait synchro");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    free_ = false;
  }
}
