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
#include <std_msgs/String.h>

namespace test_gripper
{

class TestManager
{
public:
  TestManager();

private:
  void demoCommandCallback(const std_msgs::String::ConstPtr &msg);

  ros::Publisher total_test_time_pub_;
  ros::Publisher number_of_test_pub_;
};

} // namespace test_gripper
#endif // TEST_MANAGER_H
