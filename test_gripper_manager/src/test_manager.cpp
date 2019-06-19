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

namespace test_gripper
{

TestManager::TestManager()
  : is_start_(false),
    is_ready_(false),
    current_process_(NONE)
{
  // set sequency
  ready_sequency_.push_back(MOVE_UP);
  ready_sequency_.push_back(GRASP_OFF);
  ready_sequency_.push_back(MOVE_DOWN);

  test_sequency_.push_back(GRASP_ON);
  test_sequency_.push_back(MOVE_UP);
  test_sequency_.push_back(WAIT);
  test_sequency_.push_back(MOVE_DOWN);
  test_sequency_.push_back(GRASP_OFF);

  queue_thread_ = boost::thread(boost::bind(&TestManager::queueThread, this));
  demo_thread_ = boost::thread(boost::bind(&TestManager::demoThread, this));
}

TestManager::~TestManager()
{
  queue_thread_.join();
}

void TestManager::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  total_test_time_pub_ = ros_node.advertise<std_msgs::Duration>("/demo/total_test_time", 1);
  number_of_test_pub_ = ros_node.advertise<std_msgs::Int32>("/demo/total_test_count", 1);
  movement_done_sub_ = ros_node.subscribe("/robotis/test_gripper/movement_done", 1, &TestManager::movementDoneCallback, this);
  command_sub_ = ros_node.subscribe("/demo/test_gripper/command", 1, &TestManager::demoCommandCallback, this);

  ros::WallDuration duration(0.1);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TestManager::demoThread()
{
  ros::Duration dur(0.1);
  while(ros::ok())
  {
    // check command
    switch (current_process_)
    {
    case ON_START:
      if(is_ready_ == false)
      {
        job_sequency_.clear();
        job_sequency_.assign(ready_sequency_.begin(), ready_sequency_.end());

        current_process_ = ON_PLAY;
      }
      else
      {
        current_process_ = ON_READY;
      }

      is_start_ = true;
      break;


    case ON_STOP:
      break;

    case ON_RESUME:
      break;

    case ON_READY:
      start_time_ = ros::Time::now();
      total_test_time_ = ros::Duration(0.0);

      job_sequency_.clear();
      job_sequency_.assign(test_sequency_.begin(), test_sequency_.end());
      current_process_ = ON_PLAY;
      break;

    case ON_WAIT_DONE:
      current_job_index_ += 1;
      current_process_ = ON_PLAY;
      break;


    default:
      break;

    }

    // process
    if(is_start_ == true)
    {
      // if not, is_wait
      if(current_process_ == ON_PLAY)
      {
        if(current_job_index_ <= job_sequency_.size())
        {
          // play current task
          switch(job_sequency_[current_job_index_])
          {
          case GRASP_ON:
            test_module_->graspGripper(true);
            current_process_ = ON_WAIT;
            break;

          case GRASP_OFF:
            test_module_->graspGripper(false);
            current_process_ = ON_WAIT;
            break;

          case WAIT:
            ros::Duration(3.0).sleep();
            current_process_ = ON_WAIT_DONE;
            break;

          case MOVE_UP:
            test_module_->moveUp();
            current_process_ = ON_WAIT;
            break;

          case MOVE_DOWN:
            test_module_->moveDown();
            current_process_ = ON_WAIT;
            break;

          default:
            break;
          }
        }
        else    // end of cycle
        {
          current_job_index_ = 0;

          if(is_ready_ == true)
          {
            test_count_ += 1;
            test_module_->setTestCount(test_count_);
            // publish test count
            // ...
          }
          else  // start test
          {
            is_ready_ = true;
            current_process_ = ON_READY;
          }
        }
      }
    }

    // wait 0.1sec
    dur.sleep();
  }
}

void TestManager::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Gripper";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void TestManager::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "start")
    startTest();
  else if(msg->data == "stop")
    stopTest();
}

void TestManager::movementDoneCallback(const std_msgs::String::ConstPtr &msg)
{

  if(current_process_ == ON_WAIT)
    current_process_ = ON_WAIT_DONE;
}

void TestManager::startManager()
{
  if(test_module_ != NULL)
    test_module_->setMode();
  else
    ROS_ERROR("Test module is not set to test manager.");
}

void TestManager::startTest()
{
  ROS_INFO("Start Testing");
  test_count_ = 0;
  current_job_index_ = 0;

  current_process_ = ON_START;
}

void TestManager::stopTest()
{
  ROS_INFO("Stop Testing");
  current_process_ = ON_STOP;
}

void TestManager::resumeTest()
{
  current_process_ = ON_RESUME;
}


}
