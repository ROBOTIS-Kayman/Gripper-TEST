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

#include "test_gripper_manager/test_manager.h"
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "powerxel_module/powerxel_module.h"

/* Motion Module Header */
#include "test_gripper_module/test_gripper_module.h"

using namespace test_gripper;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_gripper_manager");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    std::string robot_name  = priv_nh.param<std::string>("robot_name", "GripperTest");

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    /* Add Sensor Module */
    controller->addSensorModule((robotis_framework::SensorModule*)PowerXelModule::getInstance());

    /* Add Motion Module */
    controller->addMotionModule((robotis_framework::MotionModule*)TestGripperModule::getInstance());

    controller->startTimer();

    usleep(100 * 1000);

    // test manager
    TestManager* test_manager = new TestManager();
    test_manager->test_module_ = TestGripperModule::getInstance();
    test_manager->setRobotName(robot_name);
    test_manager->startManager();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
