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

#ifndef TEST_GRIPPER_MODULE_H_
#define TEST_GRIPPER_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#include "loadcell_idc/LoadCellState.h"

#include "joint_status.h"

namespace test_gripper
{

#define NUM_TEST_GRIPPER_JOINTS  3

class TestGripperModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<TestGripperModule>
{
private:
  int test_count_;
  bool is_error_;
  double control_cycle_sec_;
  bool get_loadcell_;
  std::string data_file_name_;
  std::string data_file_path_;
  boost::thread  queue_thread_;
  boost::thread* tra_gene_tread_;

  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher goal_torque_limit_pub_;
  ros::Publisher movement_done_pub_;

  std::map<std::string, int> joint_name_to_id_;

  /* base parameters */
  bool is_moving_;

  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd goal_joint_position_;

  sensor_msgs::JointState goal_joint_pose_msg_;
  loadcell_idc::LoadCellState loadcell_state_;

  /* movement */
  double mov_time_;
  int all_time_steps_;
  int cnt_;

  Eigen::MatrixXd goal_joint_tra_;

  void queueThread();

  void clearLoadcell() {loadcell_state_.state = 0; loadcell_state_.value = 0;}
  void loadcellStateCallback(const loadcell_idc::LoadCellState::ConstPtr &msg);
  void setJointPoseMsgCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void setCommandCallback(const std_msgs::String::ConstPtr &msg);
  void traGeneProcJointSpace();
  void setTorqueLimit();
  bool setEndTrajectory();

  bool checkTrajectory();

public:
  TestGripperModule();
  virtual ~TestGripperModule();

  /* ROS Topic Callback Functions */
  void setMode();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);

  void handleCommand(const std::string &command);
  void getLoadcell() {get_loadcell_ = true;}
  void moveUp();
  void moveUpToLoadcell();
  void moveDown();
  void graspGripper(bool is_on);
  void graspGripper(const std::string &type);
  void saveData(bool on_start, int sub_index);
  void saveStatus(std::string joint_name, std::string error_status, robotis_framework::Dynamixel *dxl);
  void setTestCount(int count) {test_count_ = count;}
  bool checkError() {return is_error_;}
  void clearError() {is_error_ = false;}
  void setDataFileName(const std::string &file_name) {data_file_name_ = file_name;}
  const std::string& getDataFilePath() {return data_file_path_;}
  const std::string& getDataFileName() {return data_file_name_;}

  const std::string currentDateTime();

  std::vector<std::string> save_data_category_;
  std::map<std::string, double> down_joint_value_;
  std::map<std::string, double> up_joint_value_;
  std::map<std::string, double> up2_joint_value_;
  std::map<std::string, double> gripper_value_;
  std::map<std::string, double> goal_joint_pose_;
  std::map<std::string, JointStatus*> joint_data_;
  std::string current_job_;
};

}

#endif /* TEST_GRIPPER_MODULE_H_ */
