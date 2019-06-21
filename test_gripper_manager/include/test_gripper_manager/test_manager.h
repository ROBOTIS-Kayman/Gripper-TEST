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

#ifndef TEST_MANAGER_H
#define TEST_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Duration.h>

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "test_gripper_module/test_gripper_module.h"

namespace test_gripper
{

class TestManager
{
public:
  enum TASK_INDEX
  {
    READYDEMO = 0,
    GRASP_ON = 1,
    MOVE_UP = 2,
    WAIT = 3,
    MOVE_DOWN = 4,
    GRASP_OFF = 5,
  };

  enum PROCESS_INDEX
  {
    NONE = 0,
    ON_START = 1,
    ON_READY = 2,
    ON_STOP = 3,
    ON_RESUME = 4,
    ON_WAIT = 5,
    ON_WAIT_DONE = 6,
    ON_PLAY = 7,
    ON_INIT = 8,
    ON_STOP_DONE = 9,
  };

  TestManager();
  ~TestManager();

  void startManager();
  void readyTest();
  void startTest();
  void stopTest();
  void startContinueTest();
  void resumeTest();
  bool getPrevTestData(std::string &save_path, int &test_count, double &test_time);
  void savePrevTestData();

  TestGripperModule* test_module_;

private:
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishCount();
  void publishTestTime();
  void demoCommandCallback(const std_msgs::String::ConstPtr &msg);
  void movementDoneCallback(const std_msgs::String::ConstPtr &msg);
  void demoThread();
  void queueThread();
  void setTimer(double sec);
  void setTimerThread(double sec);

  boost::thread  queue_thread_;
  boost::thread  demo_thread_;

  ros::Publisher status_msg_pub_;
  ros::Publisher total_test_time_pub_;
  ros::Publisher number_of_test_pub_;
  ros::Subscriber movement_done_sub_;
  ros::Subscriber command_sub_;

  std::string last_command_;
  bool is_start_, is_ready_;
  int current_process_;
  int current_job_index_;
  std::vector<int> ready_sequency_;
  std::vector<int> test_sequency_;
  std::vector<int> job_sequency_;
  ros::Time start_time_;
  ros::Duration total_test_time_;
  int test_count_;
};

} // namespace test_gripper
#endif // TEST_MANAGER_H
