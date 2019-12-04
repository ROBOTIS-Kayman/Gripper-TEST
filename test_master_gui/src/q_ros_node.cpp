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

#include "../include/test_master_gui/q_ros_node.h"

namespace test_master_gui {


QNodeTestMaster::QNodeTestMaster(int argc, char** argv)
  : init_argc_(argc),
    init_argv_(argv),
    set_end_count_(false),
    end_test_count_(0)
{

}

QNodeTestMaster::~QNodeTestMaster()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }

  wait();
}

bool QNodeTestMaster::init()
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
  ros::NodeHandle p_nh("~");

  // parsing robot list from ros parameter
  std::string robots = p_nh.param<std::string>("robot_list", "");
  boost::split(robot_list_, robots, boost::is_any_of(","));

  // initialize variable
  test_count_ = 0;
  test_time_ = ros::Duration(0.0);

  // Publisher and Subscriber
  //  test_command_pub_ = nh.advertise<std_msgs::String>("test_gripper_command", 0);
  //  test_count_sub_ = nh.subscribe("total_test_count", 1, &QNodeTestMaster::testCountCallback, this);
  //  test_time_sub_ = nh.subscribe("total_test_time", 1, &QNodeTestMaster::testTimeCallback, this);
  //  status_msg_sub_ = nh.subscribe("/robotis/status", 1, &QNodeTestMaster::statusMsgCallback, this);
  //  loadcell_sub_ = nh.subscribe("loadcell_state", 1, &QNodeTestMaster::loadcellCallback, this);

  for(auto robot_it = robot_list_.begin(); robot_it != robot_list_.end(); )
  {
    boost::algorithm::trim(*robot_it);
    if(robot_it->empty())
    {
      robot_it = robot_list_.erase(robot_it);
    }
    else
    {
      std::string topic_name = *robot_it + "/test_gripper_command";
      ros::Publisher command_pub = nh.advertise<std_msgs::String>(topic_name, 0);
      //      test_command_pub_list_.push_back(command_pub);
      test_command_pub_list_[*robot_it] = command_pub;
      robot_it++;
    }
  }

  // start thread
  start();

  return true;
}

void QNodeTestMaster::run()
{
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  Q_EMIT shutdown_ros();
}

void QNodeTestMaster::sendCommandToAll(const std::string &command)
{
  std_msgs::String command_msg;
  command_msg.data = command;

  for(auto pub_it = test_command_pub_list_.begin(); pub_it != test_command_pub_list_.end(); ++pub_it)
  {
    pub_it->second.publish(command_msg);
    //    (*pub_it).publish(command_msg);
  }
}

void QNodeTestMaster::sendCommand(const std::string &robot_name, const std::string &command)
{
  std_msgs::String command_msg;
  command_msg.data = command;

  //publish
  if(robot_name == "all")
  {
    for(auto pub_it = test_command_pub_list_.begin(); pub_it != test_command_pub_list_.end(); ++pub_it)
    {
      pub_it->second.publish(command_msg);
      //    (*pub_it).publish(command_msg);
    }
  }
  else
  {
    auto find_it = test_command_pub_list_.find(robot_name);
    if(find_it != test_command_pub_list_.end())
      find_it->second.publish(command_msg);
  }
}

void QNodeTestMaster::testCountCallback(const std_msgs::Int32::ConstPtr &msg)
{
  // store test count
  test_count_ = msg->data;

  // check to set end count
  //  if(set_end_count_ == true && test_count_ == (end_test_count_ - 1))
  if(set_end_count_ == true && test_count_ == end_test_count_)
  {
    // stop next round
    sendCommandToAll("stop_end");

    Q_EMIT clearSetEndTest();
  }

  //  if(set_end_count_ == true && test_count_ == end_test_count_)
  //  {
  //    Q_EMIT clearSetEndTest();
  //  }

  // update ui
  Q_EMIT updateTestCount(test_count_);
}

void QNodeTestMaster::testTimeCallback(const std_msgs::Duration::ConstPtr &msg)
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

void QNodeTestMaster::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
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

void QNodeTestMaster::loadcellCallback(const loadcell_idc::LoadCellState::ConstPtr &msg)
{
  std::string loadcell_state = "";
  if(msg->state == loadcell_idc::LoadCellState::UNSTABLE)
    loadcell_state = "UNSTABLE";
  else if(msg->state == loadcell_idc::LoadCellState::STABLE)
    loadcell_state = "STABLE";
  double value = msg->value;

  Q_EMIT updateLoadcell(loadcell_state, value);
}

}
