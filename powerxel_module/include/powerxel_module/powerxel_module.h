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

/* Author: Kayman */

#ifndef POWERXEL_MODULE_H
#define POWERXEL_MODULE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_framework_common/sensor_module.h"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace test_gripper
{

class PowerXelModule : public robotis_framework::SensorModule, public robotis_framework::Singleton<PowerXelModule>
{
 public:
  PowerXelModule();
  virtual ~PowerXelModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

 private:
  const bool DEBUG_PRINT;

  void queueThread();

  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishData(double current, double voltage);
  double changeScale(int32_t input) { return 0.001 * input;}
  double lowPassFilter(double alpha, double x_new, double &x_old);
  void readData(robotis_framework::Sensor *powerxel, double &current, double &voltage);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  std::map<std::string, double> previous_result_;

  /* subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher current_pub_;
  ros::Publisher voltage_pub_;

  std::string device_name_;
  int baud_rate_;

  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
};

}

#endif /* POWERXEL_MODULE_H */
