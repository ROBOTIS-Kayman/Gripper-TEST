/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdio.h>
#include "test_gripper_module/test_gripper_module.h"

using namespace test_gripper;

TestGripperModule::TestGripperModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    current_job_("none")
{
  enable_       = false;
  module_name_  = "test_gripper_module";
  control_mode_ = robotis_framework::PositionControl;

  /* gripper */
  result_["joint_1"] = new robotis_framework::DynamixelState();
  result_["joint_2"] = new robotis_framework::DynamixelState();
  result_["test_grip"] = new robotis_framework::DynamixelState();

  /* gripper */
  joint_name_to_id_["joint_1"] = 0;
  joint_name_to_id_["joint_2"] = 1;
  joint_name_to_id_["test_grip"] = 2;

  down_joint_value_["joint_1"] = 30.0;
  down_joint_value_["joint_2"] = 15.0;
  down_joint_value_["test_grip"] = 0.0;
  up_joint_value_["joint_1"] = -30.0;
  up_joint_value_["joint_2"] = 75.0;
  up_joint_value_["test_grip"] = 66.0;

  /* ----- parameter initialization ----- */
  present_joint_position_ = Eigen::VectorXd::Zero(result_.size());
  present_joint_velocity_ = Eigen::VectorXd::Zero(result_.size());
  goal_joint_position_    = Eigen::VectorXd::Zero(result_.size());
}

TestGripperModule::~TestGripperModule()
{
  queue_thread_.join();
}

void TestGripperModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&TestGripperModule::queueThread, this));
}

void TestGripperModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_torque_limit_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 1);
  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/test/movement_done", 1);

  /* subscribe topics */
//  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/test/gripper/set_mode_msg", 5,
//                                                        &TestGripperModule::setModeMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/test/gripper/joint_pose_msg", 5,
                                                          &TestGripperModule::setJointPoseMsgCallback, this);

  ros::Subscriber set_command_sub = ros_node.subscribe("/test/gripper/command", 1, &TestGripperModule::setCommandCallback, this);

  /* service */
//  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/wholebody/get_joint_pose",
//                                                                       &TestGripperModule::getJointPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TestGripperModule::setMode()
{
  //  ROS_INFO("--- Set Torque Control Mode ---");
  std_msgs::String str_msg;
  str_msg.data = module_name_;
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void TestGripperModule::setJointPoseMsgCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  goal_joint_pose_msg_ = *msg;

  if (is_moving_ == false)
  {
    setTorqueLimit();

    tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

//  movement_done_msg_.data = "gripper";

  return;
}

void TestGripperModule::setCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "start")
  {
    setMode();
    ROS_INFO("Set Mode : Test Gripper");
  }
  else if(enable_ == false)
  {
    ROS_ERROR("Module is not enabled, Command can not be executed.");
    return;
  }

  if(msg->data == "gripper_on")
  {
    graspGripper(true);
  }
  else if(msg->data == "gripper_off")
  {
    graspGripper(false);
  }
  else if(msg->data == "move_up")
  {
    moveUp();
  }
  else if(msg->data == "move_down")
  {
    moveDown();
  }

}

void TestGripperModule::traGeneProcJointSpace()
{
  mov_time_ = 1.5;
  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, result_.size() + 1);

  /* calculate joint trajectory */
  for (int dim = 0; dim < result_.size(); dim++)
  {
    double ini_value = goal_joint_position_(dim);
    double tar_value = goal_joint_position_(dim);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  //for (int dim = 0; dim < goal_joint_pose_msg_.name.size(); dim++)
  for(std::map<std::string, double>::iterator goal_it = goal_joint_pose_.begin(); goal_it != goal_joint_pose_.end(); ++goal_it)
  {
    //std::string joint_name = goal_joint_pose_msg_.name[dim];
    std::string joint_name = goal_it->first;
    int id = joint_name_to_id_[joint_name];

    double ini_value = goal_joint_position_(id);
    double tar_value = goal_it->second;

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  ROS_INFO("[start] send trajectory");
}

void TestGripperModule::setTorqueLimit()
{
  robotis_controller_msgs::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "goal_torque";

  for (int dim=0; dim<goal_joint_pose_msg_.name.size(); dim++)
  {
    std::string joint_name = goal_joint_pose_msg_.name[dim];
    int torque_limit = (int) goal_joint_pose_msg_.effort[dim];

    sync_write_msg.joint_name.push_back(joint_name);
    sync_write_msg.value.push_back(torque_limit);
  }

  goal_torque_limit_pub_.publish(sync_write_msg);
}

void TestGripperModule::setEndTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;
      cnt_ = 0;

      std_msgs::String done_msg;
      done_msg.data = current_job_;

      movement_done_pub_.publish(done_msg);
      current_job_ = "none";
    }
  }
}

void TestGripperModule::checkTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ == 0)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

    // joint space control
    for (int dim = 0; dim < result_.size(); dim++)
      goal_joint_position_(dim) = goal_joint_tra_(cnt_, dim);

    cnt_++;
  }
}

void TestGripperModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                            std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- Get Joint Data & Sensor Data-----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    // Get Joint Data
    present_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_velocity_;

    goal_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->goal_position_;
  }

  /* ----- Movement Event -----*/
  checkTrajectory();

  /* ---- Send Goal Joint Data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
  }

  /*---------- Movement End Event ----------*/
  setEndTrajectory();
}

void TestGripperModule::stop()
{
  is_moving_ = false;

  return;
}

bool TestGripperModule::isRunning()
{
  return is_moving_;
}

void TestGripperModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Gripper";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void TestGripperModule::moveUp()
{
  if(is_moving_ == true)
  {
    ROS_ERROR_STREAM("It's busy now, try again. : " << current_job_);
    return;
  }

  current_job_ = "move_up";

  goal_joint_pose_.clear();
  goal_joint_pose_["joint_1"] = up_joint_value_["joint_1"] * M_PI / 180.0;
  goal_joint_pose_["joint_2"] = up_joint_value_["joint_2"] * M_PI / 180.0;

  tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
  delete tra_gene_tread_;
}

void TestGripperModule::moveDown()
{
  if(is_moving_ == true)
  {
    ROS_ERROR_STREAM("It's busy now, try again. : " << current_job_);
    return;
  }

  current_job_ = "move_down";

  goal_joint_pose_.clear();
  goal_joint_pose_["joint_1"] = down_joint_value_["joint_1"] * M_PI / 180.0;
  goal_joint_pose_["joint_2"] = down_joint_value_["joint_2"] * M_PI / 180.0;

  tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
  delete tra_gene_tread_;
}

void TestGripperModule::graspGripper(bool is_on)
{
  if(is_moving_ == true)
  {
    ROS_ERROR_STREAM("It's busy now, try again. : " << current_job_);
    return;
  }

  if(is_on == true)
  {
    current_job_ = "grasp_on";

    goal_joint_pose_.clear();
    goal_joint_pose_["test_grip"] = up_joint_value_["test_grip"] * M_PI / 180.0;
  }
  else
  {
    current_job_ = "grasp_off";
    goal_joint_pose_["test_grip"] = down_joint_value_["test_grip"] * M_PI / 180.0;
  }

  tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
  delete tra_gene_tread_;
}
