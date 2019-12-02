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

#ifndef Q_ROS_NODE_H
#define Q_ROS_NODE_H

#include <cstdio>
#include <fstream>
#include <QThread>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Duration.h>

#include "yaml-cpp/yaml.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "loadcell_idc/LoadCellState.h"

namespace test_gripper_gui {

class QNodeTestGriper : public QThread
{
  Q_OBJECT

public:
  QNodeTestGriper(int argc, char** argv);
  virtual ~QNodeTestGriper();

  bool init();
  void run();
  void sendCommand(const std::string &command);
  void setEndCount(bool is_set, int end_count) {set_end_count_ = is_set; end_test_count_ = end_count; }
  bool getRobotName(std::string &robot_name) { robot_name = robot_name_; return !robot_name_.empty(); }

public Q_SLOTS:
//  void changeControlRobot(int index);

Q_SIGNALS:
  void shutdown_ros();
  void log(const std::string& message);
  void updateTestTime(const std::string &time);
  void updateTestCount(int count);
  void clearSetEndTest();
  void updateLoadcell(const std::string &state, double value);

private:
  int init_argc_;
  char** init_argv_;

  int test_count_;
  ros::Duration test_time_;
  bool set_end_count_;
  int end_test_count_;
  std::string robot_name_;

  ros::Publisher test_command_pub_;
  ros::Subscriber test_count_sub_;
  ros::Subscriber test_time_sub_;
  ros::Subscriber status_msg_sub_;
  ros::Subscriber loadcell_sub_;

  void testCountCallback(const std_msgs::Int32::ConstPtr &msg);
  void testTimeCallback(const std_msgs::Duration::ConstPtr &msg);
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
  void loadcellCallback(const loadcell_idc::LoadCellState::ConstPtr &msg);
};

}









#endif // Q_ROS_NODE_H
