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

#include <stdio.h>

#include "powerxel_module/powerxel_module.h"

namespace test_gripper
{

PowerXelModule::PowerXelModule()
  : control_cycle_msec_(8),
    DEBUG_PRINT(false)
{
  module_name_ = "powerxel_module";  // set unique module name

  result_["current"] = 0.0;
  result_["voltage"] = 0.0;

  previous_result_["current"] = 0.0;
  previous_result_["voltage"] = 0.0;
}

PowerXelModule::~PowerXelModule()
{
  queue_thread_.join();
}

void PowerXelModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  auto sensor_it = robot->sensors_.find("powerxel");
  if(sensor_it != robot->sensors_.end())
  {
    std::string device_name = sensor_it->second->port_name_;
    int protocol_version = sensor_it->second->protocol_version_;

    port_handler_ = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler(device_name.c_str());
    bool set_port_result = port_handler_->setBaudRate(1000000);
    if (set_port_result == false)
    {
      ROS_ERROR("Error Set port");
      return;
    }
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  }

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&PowerXelModule::queueThread, this));
}

void PowerXelModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  //  /* publisher */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);

  if(DEBUG_PRINT == true)
  {
    current_pub_ = ros_node.advertise<std_msgs::Float64>("/robotis/powerxel/current", 1);
    voltage_pub_ = ros_node.advertise<std_msgs::Float64>("/robotis/powerxel/voltage", 1);
  }

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void PowerXelModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                             std::map<std::string, robotis_framework::Sensor *> sensors)
{
  auto sensor_it = sensors.find("powerxel");
  if(sensor_it == sensors.end())
    return;

  double low_pass_filter_ratio = 0.4;
  double current = 0, voltage = 0;

  readData(sensor_it->second, current, voltage);

  result_["current"] = lowPassFilter(low_pass_filter_ratio, current, previous_result_["current"]);
  result_["voltage"] = lowPassFilter(low_pass_filter_ratio, voltage, previous_result_["voltage"]);

  if(DEBUG_PRINT == true)
    publishData(result_["current"], result_["voltage"]);
}

void PowerXelModule::readData(robotis_framework::Sensor* powerxel, double& current, double& voltage)
{
  uint32_t raw_voltage = 0;
  uint32_t raw_current = 0;
  int id = powerxel->id_;
  //  int result = packet_handler_->read4ByteTxRx(port_handler_, powerxel->id_, powerxel->ctrl_table_["voltage"]->address_, &raw_current);

  uint8_t data_read[8] = {0};
  int result = packet_handler_->readTxRx(port_handler_, id, powerxel->ctrl_table_["voltage"]->address_, 8, data_read);
  if (result == COMM_SUCCESS)
  {
    raw_voltage = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
    raw_current = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[4], data_read[5]), DXL_MAKEWORD(data_read[6], data_read[7]));
  }

  if(result != COMM_SUCCESS)
    ROS_ERROR_STREAM("ERROR : " << powerxel->port_name_ << ", " << ", id : " << id << ", add : " << powerxel->ctrl_table_["voltage"]->address_);

  voltage = changeScale(raw_voltage);
  current = changeScale(raw_current);

  //  uint8_t data_read[4] = {0};
  //  int result = readTxRx(port, id, address, 4, data_read, error);
  //  if (result == COMM_SUCCESS)
  //    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
  //  return result;
}

void PowerXelModule::handleVoltage(double present_volt)
{
  //  double voltage_ratio = 0.4;
  //  previous_volt_ =
  //      (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  //  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  //  {
  //    // check last publised time
  //    ros::Time now = ros::Time::now();
  //    ros::Duration dur = now - last_msg_time_;
  //    if (dur.sec < 1)
  //      return;

  //    last_msg_time_ = now;

  //    present_volt_ = previous_volt_;
  //    std::stringstream log_stream;
  //    log_stream << "Present Volt : " << present_volt_ << "V";
  //    publishStatusMsg(
  //        (present_volt_ < 11 ?
  //            robotis_controller_msgs::StatusMsg::STATUS_WARN : robotis_controller_msgs::StatusMsg::STATUS_INFO),
  //        log_stream.str());
  //    ROS_INFO_COND(DEBUG_PRINT, "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  //  }
}

void PowerXelModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void PowerXelModule::publishData(double current, double voltage)
{
  std_msgs::Float64 current_msg;
  std_msgs::Float64 voltage_msg;

  current_msg.data = current;
  voltage_msg.data = voltage;

  current_pub_.publish(current_msg);
  voltage_pub_.publish(voltage_msg);
}

double PowerXelModule::lowPassFilter(double alpha, double x_new, double &x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;

  return filtered_value;
}

}
