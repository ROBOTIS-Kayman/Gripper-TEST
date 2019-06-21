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

#include "../include/test_gripper_gui/q_ros_node.h"

namespace test_gripper_gui {


QNodeTestGriper::QNodeTestGriper(int argc, char** argv)
  : init_argc_(argc),
    init_argv_(argv)
{

}

QNodeTestGriper::~QNodeTestGriper()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }

  wait();
}

bool QNodeTestGriper::init()
{
  // init ros
  ros::init(init_argc_, init_argv_, ROS_PACKAGE_NAME);

  if(ros::master::check() == false)
  {
    ROS_ERROR("Not found ros master!!!");
    return false;
  }

  ros::start();

  ros::NodeHandle nh;

  // initialize variable
  test_count_ = 0;
  test_time_ = ros::Duration(0.0);

  // Publisher and Subscriber
  test_command_pub_ = nh.advertise<std_msgs::String>("/demo/test_gripper/command", 0);
  test_count_sub_ = nh.subscribe("/demo/total_test_count", 1, &QNodeTestGriper::testCountCallback, this);
  test_time_sub_ = nh.subscribe("/demo/total_test_time", 1, &QNodeTestGriper::testTimeCallback, this);
  status_msg_sub_ = nh.subscribe("/robotis/status", 1, &QNodeTestGriper::statusMsgCallback, this);

  // start thread
  start();

  return true;
}

void QNodeTestGriper::run()
{
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  Q_EMIT shutdown_ros();
}

void QNodeTestGriper::sendCommand(const std::string &command)
{
   std_msgs::String command_msg;
   command_msg.data = command;

   //publish
   test_command_pub_.publish(command_msg);
}

void QNodeTestGriper::testCountCallback(const std_msgs::Int32::ConstPtr &msg)
{
  // store test count
  test_count_ = msg->data;

  // update ui
  Q_EMIT updateTestCount(test_count_);
}

void QNodeTestGriper::testTimeCallback(const std_msgs::Duration::ConstPtr &msg)
{
  // store test time
  test_time_ = msg->data;

  // update ui
  int total_sec = test_time_.toSec();
  int total_min = total_sec / 60;
  int sec = total_sec % 60;
  int min = total_min % 60;
  int hour = total_min / 60;

  char c_time [12];
  sprintf(c_time, "%04d:%02d:%02d", hour, min, sec);
  std::string time(c_time);

  Q_EMIT updateTestTime(time);
}

void QNodeTestGriper::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  std::string status_msg;
  switch(msg->type)
  {
  case robotis_controller_msgs::StatusMsg::STATUS_UNKNOWN:
    status_msg = "[UNKNOWN] ";
    break;

  case robotis_controller_msgs::StatusMsg::STATUS_INFO:
    status_msg = "[INFO] ";
    break;

  case robotis_controller_msgs::StatusMsg::STATUS_WARN:
    status_msg = "[WARN] ";
    break;

  case robotis_controller_msgs::StatusMsg::STATUS_ERROR:
    status_msg = "[ERROR] ";
    break;

  default:
    break;
  }

  status_msg = status_msg + msg->status_msg;

  Q_EMIT log(status_msg);
}

}
