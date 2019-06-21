/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

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
//  void navigationMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
//  void initialPoseMsgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
//  const std::map<int, std::string>& getParameters() { return robot_list_; }
//  const std::map<std::string, geometry_msgs::Pose2D>& getStartPositionList() { return start_list_; }
//  const std::map<std::string, geometry_msgs::Pose2D>& getGoalPositionList() { return goal_list_; }
//  void visualizePositionMarker(int type, bool clear = false);
//  bool saveCurrentPosition(int type, const std::string& saved_name);
//  bool deleteCurrentPosition(int type, const std::string& saved_name);
//  void publishNavMsg(int type, const std::string goal_name);

public Q_SLOTS:
//  void changeControlRobot(int index);

Q_SIGNALS:
  void shutdown_ros();
  void log(const std::string& message);
  void updateTestTime(const std::string &time);
  void updateTestCount(int count);

private:
  int init_argc_;
  char** init_argv_;

  int test_count_;
  ros::Duration test_time_;

  ros::Publisher test_command_pub_;
  ros::Subscriber test_count_sub_;
  ros::Subscriber test_time_sub_;
  ros::Subscriber status_msg_sub_;

  void testCountCallback(const std_msgs::Int32::ConstPtr &msg);
  void testTimeCallback(const std_msgs::Duration::ConstPtr &msg);
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
};

}









#endif // Q_ROS_NODE_H
