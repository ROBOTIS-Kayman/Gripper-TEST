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
    current_job_("none"),
    test_count_(0),
    is_error_(false),
    get_loadcell_(false)
{
  enable_       = false;
  module_name_  = "test_gripper_module";
  control_mode_ = robotis_framework::PositionControl;

  data_file_name_.clear();

  /* gripper */
  result_["joint_1"] = new robotis_framework::DynamixelState();
  result_["joint_2"] = new robotis_framework::DynamixelState();
  result_["gripper"] = new robotis_framework::DynamixelState();

  joint_data_["joint_1"] = new JointStatus("joint_1");
  joint_data_["joint_2"] = new JointStatus("joint_2");
  joint_data_["gripper"] = new JointStatus("gripper");

  // set base current to check error : 5kg
  joint_data_["joint_1"]->base_current_ = 1200; // 5kg : 3000;
  joint_data_["joint_1"]->check_current_task_ = "move_up_2, move_down_1";
  joint_data_["gripper"]->base_current_ = 600;
  joint_data_["gripper"]->check_current_task_ = "grasp_on_2, move_up_1, move_up_2, move_down_1, move_down_2, grasp_off_1";

  save_data_category_.push_back("hardware_error_status");
  save_data_category_.push_back("goal_position");
  save_data_category_.push_back("present_position");
  save_data_category_.push_back("present_current");
  save_data_category_.push_back("present_temperature");

  for(std::map<std::string, JointStatus*>::iterator it = joint_data_.begin(); it != joint_data_.end(); ++it)
  {
    it->second->data_list_.assign(save_data_category_.begin(), save_data_category_.end());
  }

  /* gripper */
  joint_name_to_id_["joint_1"] = 0;
  joint_name_to_id_["joint_2"] = 1;
  joint_name_to_id_["gripper"] = 2;

  // ready to grasp
  down_joint_value_["joint_1"] = 30.0;
  down_joint_value_["joint_2"] = 15.0;
  down_joint_value_["gripper"] = 0.0;   // off

  // hold on
  up_joint_value_["joint_1"] = -30.0;
  up_joint_value_["joint_2"] = 75.0;
  up_joint_value_["gripper"] = 66.0;    // on

  // loadcell
  up2_joint_value_["joint_1"] = 30.0;
  up2_joint_value_["joint_2"] = -75.0;
  up2_joint_value_["gripper"] = 0.0;    // off

  // gripper
  gripper_value_["grasp_on"] = 66.0;
  gripper_value_["grasp_off"] = 0.0;
  gripper_value_["grasp_on_loadcell"] = 55.0;

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
  data_file_path_ = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/";

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
  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/test_gripper/movement_done", 1);

  /* subscribe topics */
  //  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/test/gripper/set_mode_msg", 5,
  //                                                        &TestGripperModule::setModeMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/test_gripper/gripper/joint_pose_msg", 5,
                                                          &TestGripperModule::setJointPoseMsgCallback, this);

  ros::Subscriber set_command_sub = ros_node.subscribe("/robotis/test_gripper/command", 1, &TestGripperModule::setCommandCallback, this);

  ros::Subscriber loadcell_sub = ros_node.subscribe("loadcell_state", 1, &TestGripperModule::loadcellStateCallback, this);

  /* service */
  //  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/wholebody/get_joint_pose",
  //                                                                       &TestGripperModule::getJointPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TestGripperModule::setMode()
{
  ROS_INFO("Set Mode : Test Gripper");

  std_msgs::String str_msg;
  str_msg.data = module_name_;
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void TestGripperModule::loadcellStateCallback(const loadcell_idc::LoadCellState::ConstPtr &msg)
{
  if(get_loadcell_ == false)
    return;

  if(msg->state == loadcell_idc::LoadCellState::STABLE)
  {
    loadcell_state_ = *msg;
    get_loadcell_ = false;
  }
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
  // call handle command
  handleCommand(msg->data);
}

void TestGripperModule::traGeneProcJointSpace()
{
  mov_time_ = 1.5;
  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, result_.size() + 1);

  /* calculate joint trajectory */
  for(std::map<std::string, robotis_framework::DynamixelState *>::iterator result_it = result_.begin(); result_it != result_.end(); ++result_it)
  {
    //std::string joint_name = goal_joint_pose_msg_.name[dim];
    std::string joint_name = result_it->first;
    int id = joint_name_to_id_[joint_name];

    double ini_value = goal_joint_position_(id);
    double tar_value = goal_joint_position_(id);

    std::map<std::string, double>::iterator goal_it = goal_joint_pose_.find(joint_name);
    if(goal_it != goal_joint_pose_.end())
      tar_value = goal_it->second;

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  //  for (int dim = 0; dim < result_.size(); dim++)
  //  {
  //    double ini_value = goal_joint_position_(dim);
  //    double tar_value = goal_joint_position_(dim);

  //    Eigen::MatrixXd tra =
  //        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
  //                                              tar_value , 0.0 , 0.0 ,
  //                                              control_cycle_sec_, mov_time_);

  //    goal_joint_tra_.block(0, dim, all_time_steps_, 1) = tra;
  //  }

  //  //for (int dim = 0; dim < goal_joint_pose_msg_.name.size(); dim++)
  //  for(std::map<std::string, double>::iterator goal_it = goal_joint_pose_.begin(); goal_it != goal_joint_pose_.end(); ++goal_it)
  //  {
  //    //std::string joint_name = goal_joint_pose_msg_.name[dim];
  //    std::string joint_name = goal_it->first;
  //    int id = joint_name_to_id_[joint_name];

  //    double ini_value = goal_joint_position_(id);
  //    double tar_value = goal_it->second;

  //    Eigen::MatrixXd tra =
  //        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
  //                                              tar_value , 0.0 , 0.0 ,
  //                                              control_cycle_sec_, mov_time_);

  //    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  //  }

  cnt_ = 0;
  is_moving_ = true;

  ROS_INFO_STREAM("[ready] make trajectory : " << current_job_);
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

bool TestGripperModule::setEndTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO_STREAM("[end] send trajectory : " << current_job_);

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory : " + current_job_);

      is_moving_ = false;
      cnt_ = 0;

      return true;
    }
  }

  return false;
}

bool TestGripperModule::checkTrajectory()
{
  bool on_start = false;
  if (is_moving_ == true)
  {
    if (cnt_ == 0)
    {
      ROS_INFO_STREAM("[start] send trajectory : " << current_job_);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory : " + current_job_);
      on_start = true;
    }

    // joint space control
    for (int dim = 0; dim < result_.size(); dim++)
      goal_joint_position_(dim) = goal_joint_tra_(cnt_, dim);

    cnt_++;
  }

  return on_start;
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

    //    saveStatus(joint_name, current_job_, dxl);
  }

  /* ----- Movement Event -----*/
  bool is_start = checkTrajectory();

  /* ---- Send Goal Joint Data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
  }

  /*---------- Movement End Event ----------*/
  bool is_finished = setEndTrajectory();

  /*---------- Check Error and Save data ----------*/
  if(is_finished == true || is_start == true)
  {
    is_error_ = false;
    int sub_index = is_start ? 1 : 2;
    // store data to save
    // loop dxl to find joints to check overload limit
    for (auto& it : dxls)
    {
      std::string joint_name = it.first;
      robotis_framework::Dynamixel* dxl = it.second;

      std::string error_status = "";
      // check position error : above 5.0 deg
      double diff_position = fabs(dxl->dxl_state_->goal_position_ - dxl->dxl_state_->present_position_) * 180.0 / M_PI;
      if(diff_position > 5.0)
      {
        error_status = "position error";
        is_error_ = true;
      }

      // check current error when testing
      if(test_count_ > 0)
      {
        std::string sub_task = current_job_;
        if(is_start == true)
          sub_task = current_job_ + "_1";
        if(is_finished == true)
          sub_task = current_job_ + "_2";

        JointStatus *joint_status = joint_data_[joint_name];
        if(joint_status->check_current_task_.find(sub_task) != std::string::npos)
        {
          std::string present_current_name = dxl->present_current_item_->item_name_;
          std::map<std::string, uint32_t>::iterator current_it = dxl->dxl_state_->bulk_read_table_.find(present_current_name);
          if(current_it != dxl->dxl_state_->bulk_read_table_.end())
          {
            uint16_t present_current_data = current_it->second;
            int16_t present_current = present_current_data;

            if(abs(present_current) < joint_status->base_current_)
            {
              if(error_status != "")
                error_status = error_status + "|current error";
              else
                error_status = "current error";

              is_error_ = true;
            }
          }
        }
      }

      saveStatus(joint_name, error_status, dxl);
    }

    saveData(false, sub_index);
  }

  // publish movement done msg
  if(is_finished == true)
  {
    // send movement done msg
    std_msgs::String done_msg;
    done_msg.data = current_job_;

    movement_done_pub_.publish(done_msg);
    current_job_ = "none";
    //    test_count_ += 1;
  }
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

void TestGripperModule::handleCommand(const std::string &command)
{

  if(command == "start")
  {
    setMode();
  }
  else if(enable_ == false)
  {
    ROS_ERROR("Module is not enabled, Command can not be executed.");
    return;
  }

  if(command == "gripper_on")
  {
    graspGripper(true);
  }
  else if(command == "gripper_off")
  {
    graspGripper(false);
  }
  else if(command == "move_up")
  {
    moveUp();
  }
  else if(command == "move_up_to_loadcell")
  {
    moveUpToLoadcell();
  }
  else if(command == "move_down")
  {
    moveDown();
  }
  else if(command == "save_start")
  {
    saveData(true, 0);
  }
  else if(command == "save_continue")
  {
    saveData(false, 0);
  }
  else if(command == "get_loadcell")
  {
    getLoadcell();
  }
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

void TestGripperModule::moveUpToLoadcell()
{
  if(is_moving_ == true)
  {
    ROS_ERROR_STREAM("It's busy now, try again. : " << current_job_);
    return;
  }

  current_job_ = "move_up2";

  goal_joint_pose_.clear();
  goal_joint_pose_["joint_1"] = up2_joint_value_["joint_1"] * M_PI / 180.0;
  goal_joint_pose_["joint_2"] = up2_joint_value_["joint_2"] * M_PI / 180.0;

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
    goal_joint_pose_["gripper"] = gripper_value_["grasp_on"] * M_PI / 180.0;
  }
  else
  {
    current_job_ = "grasp_off";
    goal_joint_pose_["gripper"] = gripper_value_["grasp_off"] * M_PI / 180.0;
  }

  tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
  delete tra_gene_tread_;
}

void TestGripperModule::graspGripper(const std::string &type)
{
  if(is_moving_ == true)
  {
    ROS_ERROR_STREAM("It's busy now, try again. : " << current_job_);
    return;
  }

  if(gripper_value_.find(type) == gripper_value_.end())
    return;

  current_job_ = type;

  goal_joint_pose_.clear();
  goal_joint_pose_["gripper"] = gripper_value_[type] * M_PI / 180.0;

  tra_gene_tread_ = new boost::thread(boost::bind(&TestGripperModule::traGeneProcJointSpace, this));
  delete tra_gene_tread_;
}

void TestGripperModule::saveData(bool on_start, int sub_index)
{
  if(test_count_ < 0)
    return;

  std::ofstream data_file;
  if(on_start == true || data_file_name_.empty())
  {
    data_file_name_ = data_file_path_ + currentDateTime() + ".csv";
    data_file.open (data_file_name_, std::ofstream::out | std::ofstream::app);

    // save index
    data_file << "index,job,sub_index,loadcell,";
    for (auto& it : joint_data_)
    {
      data_file << it.first << ",";
      for(auto& item_it : it.second->data_list_)
        data_file << item_it << ",";
    }
    data_file << std::endl;
  }
  else
    data_file.open (data_file_name_, std::ofstream::out | std::ofstream::app);

  // save data
  data_file << test_count_ << "," << current_job_ << "," << sub_index << "," << loadcell_state_.value << ",";
  clearLoadcell();
  for (auto& it : joint_data_)
  {
    data_file << it.second->joint_status_ << ",";
    for(auto& item_it : it.second->data_value_)
      data_file << item_it << "," ;
  }
  data_file << std::endl;

  data_file.close();
  ROS_INFO_STREAM("Saved : " << data_file_name_);
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Saved the monitoring data.");
}

const std::string TestGripperModule::currentDateTime()
{
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  //  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct); // YYYY-MM-DD.HH:mm:ss
  strftime(buf, sizeof(buf), "%Y-%m-%d.%H-%M-%S", &tstruct); // YYYY-MM-DD.HH-mm-ss

  return buf;
}

void TestGripperModule::saveStatus(std::string joint_name, std::string error_status, robotis_framework::Dynamixel *dxl)
{
  JointStatus *joint_status = joint_data_[joint_name];

  joint_status->joint_status_ = error_status;
  joint_status->data_value_.clear();

  for (auto& it : joint_status->data_list_)
  {
    std::string item_name = it;

    std::map<std::string, robotis_framework::ControlTableItem *>::iterator item_it = dxl->ctrl_table_.find(item_name);
    if(item_it != dxl->ctrl_table_.end())
    {
      uint8_t length = item_it->second->data_length_;

      if(length == 1)
      {
        uint8_t uint_data = dxl->dxl_state_->bulk_read_table_[item_name];
        int8_t data = uint_data;
        joint_status->data_value_.push_back(data);
      }
      else if(length == 2)
      {
        uint16_t uint_data = dxl->dxl_state_->bulk_read_table_[item_name];
        int16_t data = uint_data;
        joint_status->data_value_.push_back(data);
      }
      else
      {
        uint32_t uint_data = dxl->dxl_state_->bulk_read_table_[item_name];
        int32_t data = uint_data;
        joint_status->data_value_.push_back(data);
      }
    }
    else
    {
      joint_status->data_value_.push_back(0);
    }
  }

  //  for(std::map<std::string, JointStatus*>::iterator it = joint_data_.begin(); it != joint_data_.end(); ++it)
  //  {
  //    it->second->data_list_.push_back("hardware_error_status");
  //    it->second->data_list_.push_back("goal_position");
  //    it->second->data_list_.push_back("present_position");
  //    it->second->data_list_.push_back("present_current");
  //    it->second->data_list_.push_back("present_temperature");
  //  }
}
