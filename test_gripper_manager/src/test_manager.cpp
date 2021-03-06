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
{
  TestManager("GripperTest");
}

TestManager::TestManager(const std::string &robot_name)
  : robot_name_(robot_name),
    is_start_(false),
    is_ready_(false),
    loadcell_test_(false),
    current_process_(NONE),
    is_loadcell_task_(false)
{
  // set sequency
  ready_sequency_.push_back(MOVE_UP_READY);
  ready_sequency_.push_back(GRASP_OFF);
  ready_sequency_.push_back(MOVE_DOWN);

  test_sequency_.push_back(GRASP_ON);
  test_sequency_.push_back(MOVE_UP);
  test_sequency_.push_back(WAIT);
  test_sequency_.push_back(MOVE_DOWN);
  test_sequency_.push_back(GRASP_OFF);

  test_loadcell_sequency_.push_back(MOVE_UP_TO_LOADCELL);
  test_loadcell_sequency_.push_back(GRASP_ON_LOADCELL);
  test_loadcell_sequency_.push_back(WAIT_FOR_LOADCELL);
  test_loadcell_sequency_.push_back(GET_LOADCELL);
  test_loadcell_sequency_.push_back(GRASP_OFF);
  test_loadcell_sequency_.push_back(MOVE_DOWN_FROM_LOADCELL);

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
  total_test_time_pub_ = ros_node.advertise<std_msgs::Duration>("total_test_time", 1);
  number_of_test_pub_ = ros_node.advertise<std_msgs::Int32>("total_test_count", 1);
  movement_done_sub_ = ros_node.subscribe("/robotis/test_gripper/movement_done", 1, &TestManager::movementDoneCallback, this);
  command_sub_ = ros_node.subscribe("test_gripper_command", 1, &TestManager::demoCommandCallback, this);

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
    case ON_INIT:
      is_ready_ = false;
      job_sequency_.clear();
      job_sequency_.assign(ready_sequency_.begin(), ready_sequency_.end());
      ROS_INFO("Job is set by being ready.");

      current_process_ = ON_PLAY;
      test_module_->setTestCount(-1);

      is_start_ = true;
      break;

    case ON_START:
      if(is_ready_ == false)
      {
        job_sequency_.clear();
        job_sequency_.assign(ready_sequency_.begin(), ready_sequency_.end());
        ROS_INFO("Job is set by being ready.");

        current_process_ = ON_PLAY;
        test_module_->setTestCount(-1);
      }
      else
      {
        current_process_ = ON_READY;
      }

      is_start_ = true;
      break;


    case ON_STOP:
      if(is_start_ == true)
        is_start_ = false;
      break;

    case ON_STOP_DONE:
      current_job_index_ += 1;
      current_process_ = ON_STOP;
      break;

    case ON_RESUME:
      start_time_ = ros::Time::now();
      current_process_ = ON_PLAY;

      is_start_ = true;
      break;

    case ON_READY:
      start_time_ = ros::Time::now();

      job_sequency_.clear();
      job_sequency_.assign(test_sequency_.begin(), test_sequency_.end());
      current_process_ = ON_PLAY;
      ROS_INFO("Job is set by playing test.");
      test_module_->setTestCount(test_count_);
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
        if(current_job_index_ < job_sequency_.size())
        {
          // first task of testing
          if(current_job_index_ == 0)
          {

          }

          // check error
          if(test_module_->checkError() == true)
          {
            ROS_ERROR("Error occured!!!");
            stopTest(true);
            continue;
          }

          // publish test_count in first
          if(is_ready_ == true && current_job_index_ == 0)
          {
            // publish test count
            if(is_loadcell_task_ == false)
              test_count_ += 1;
            test_module_->setTestCount(test_count_);
            publishCount();

            // changed save file name per 1000 times
            if(test_count_ % 1000 == 1 && is_loadcell_task_ == false)
              test_module_->setDataFileName("");
          }

          // play current task
          switch(job_sequency_[current_job_index_])
          {
          case GRASP_ON:
            test_module_->graspOnOffGripper(true);
            break;

          case GRASP_OFF:
            test_module_->graspOnOffGripper(false);
            break;

          case GRASP_ON_LOADCELL:
            test_module_->graspGripper("grasp_on_loadcell");
            break;

          case WAIT:
            setTimer(3.0);
            break;

          case WAIT_FOR_LOADCELL:
            setTimer(3.0);
            break;

          case GET_LOADCELL:
            test_module_->getLoadcell();
            setTimer(1.0);
            break;

          case MOVE_UP:
            test_module_->moveUp();
            break;

          case MOVE_UP_TO_LOADCELL:
            test_module_->moveUpToLoadcell();
            break;

          case MOVE_DOWN:
            test_module_->moveDown();
            break;

          case MOVE_DOWN_FROM_LOADCELL:
            test_module_->moveDownFromLoadcell();
            break;

          case MOVE_UP_READY:
            test_module_->moveUpforReady();
            break;

          default:
            ROS_WARN("Invalid Task");
            current_process_ = ON_WAIT_DONE;
            continue;
          }

          // wait done.
          current_process_ = ON_WAIT;
        }
        else    // end of cycle
        {
          current_job_index_ = 0;

          if(is_ready_ == true)
          {
            // save test data
            savePrevTestData();

            // check time to test loadcell
            if((loadcell_test_ == true ) && ((test_count_ % LOADCELLTASK) == 0))
            {
              if(is_loadcell_task_ == false)
              {
                // start testing of loadcell
                job_sequency_.clear();
                job_sequency_.assign(test_loadcell_sequency_.begin(), test_loadcell_sequency_.end());
                is_loadcell_task_ = true;
              }
              else
              {
                // go back to gripper testing
                job_sequency_.clear();
                job_sequency_.assign(test_sequency_.begin(), test_sequency_.end());
                is_loadcell_task_ = false;
              }
            }

            // check scheduled to stop when normal test(not testing using loadcell)
            if(last_command_ == "stop_end" && is_loadcell_task_ == false)
              stopTest();
          }
          else  // "ready" or "start" or "start_continue" command
          {
            is_ready_ = true;

            if(last_command_ == "start" || last_command_ == "start_continue")
              current_process_ = ON_READY;
            else
            {
              is_start_ = false;
              current_process_ = NONE;
            }
          }
        }
      }
    }

    // wait 0.1sec
    dur.sleep();

    // publish testing time
    if(last_command_ == "start" || last_command_ == "resume" || last_command_ == "start_continue")
      publishTestTime();
  }
}

void TestManager::setTimerThread(double sec)
{
  ROS_INFO_STREAM("Wait for " << sec);
  ros::Duration(sec).sleep();

  if(current_process_ == ON_WAIT)
    current_process_ = ON_WAIT_DONE;
  else if(current_process_ == ON_STOP)
    current_process_ = ON_STOP_DONE;
}

void TestManager::setTimer(double sec)
{
  boost::thread timer_thread = boost::thread(boost::bind(&TestManager::setTimerThread, this, sec));
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

void TestManager::publishCount()
{
  std_msgs::Int32 test_count_msg;
  test_count_msg.data = test_count_;

  if(test_count_ > 0)
    number_of_test_pub_.publish(test_count_msg);
}

void TestManager::publishTestTime()
{
  if(is_start_ == false || is_ready_ == false)
    return;

  std_msgs::Duration total_test_time_msg;
  total_test_time_msg.data = (ros::Time::now() - start_time_) + total_test_time_;

  total_test_time_pub_.publish(total_test_time_msg);
}

void TestManager::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  bool result = false;

  if(msg->data == "start")
    result = startTest();
  else if(msg->data == "start_continue")
    result = startContinueTest();
  else if(msg->data == "stop")
    result = stopTest();
  else if(msg->data == "ready")
    result = readyTest();
  else if(msg->data == "resume")
    result = resumeTest();
  else if(msg->data == "stop_end")
    result = true;

  if(result == true)
    last_command_ = msg->data;
}

void TestManager::movementDoneCallback(const std_msgs::String::ConstPtr &msg)
{
  if(current_process_ == ON_WAIT)
    current_process_ = ON_WAIT_DONE;
  else if(current_process_ == ON_STOP)
    current_process_ = ON_STOP_DONE;
}

void TestManager::startManager()
{
  if(test_module_ != NULL)
  {
//    test_module_->setMode();
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();
    controller->setCtrlModule(test_module_->getModuleName());
  }
  else
    ROS_ERROR("Test module is not set to test manager.");
}

bool TestManager::readyTest()
{
  if(is_start_ == true)
  {
    ROS_INFO("Alread started testing.");
    return false;
  }

  ROS_INFO("Ready Testing");
  test_count_ = -1;
  current_job_index_ = 0;

  current_process_ = ON_INIT;

  return true;
}

bool TestManager::startTest()
{
  if(is_start_ == true)
  {
    ROS_INFO("Alread started testing.");
    return false;
  }

  ROS_INFO("Start Testing");

  // check to exist the saved file.
  std::string save_path;
  int test_count = 0;
  double test_time = 0.0;

  bool result = getPrevTestData(save_path, test_count, test_time);
  if(result == false)
  {
    test_module_->setDataFileName("");
    total_test_time_ = ros::Duration(0.0);
    test_count_ = 0;
  }
  else
  {
    ROS_WARN("The saved file is found. Test will be continued.");

    test_module_->setDataFileName(save_path);
    test_count_ = test_count;
    total_test_time_ = ros::Duration(test_time);
  }

  current_job_index_ = 0;

  current_process_ = ON_START;

  return true;
}

bool TestManager::stopTest(bool is_break)
{
  if(is_start_ == false)
  {
    ROS_INFO("Alread stopped testing");
    return false;
  }

  ROS_INFO("Stop Testing");
  current_process_ = ON_STOP;
  is_loadcell_task_ = false;
  test_module_->clearError();
  total_test_time_ = (ros::Time::now() - start_time_) + total_test_time_;
  savePrevTestData();

  if(is_break == true)
    is_ready_ = false;

  return true;
}

bool TestManager::resumeTest()
{
  if(is_start_ == true)
  {
    ROS_INFO("Alread started testing.");
    return false;
  }

  ROS_INFO("Resume Tesing");
  current_process_ = ON_RESUME;

  return true;
}

bool TestManager::startContinueTest()
{
  if(is_start_ == true)
  {
    ROS_INFO("Alread started testing.");
    return false;
  }

  // check last testing file.

  ROS_INFO("Start Testing continue");

  std::string save_path;
  int test_count = 0;
  double test_time = 0.0;

  bool result = getPrevTestData(save_path, test_count, test_time);
  if(result == false)
  {
    ROS_WARN("It can not be started continue. It will start in first.");

    total_test_time_ = ros::Duration(0.0);
    test_count_ = 0;
  }
  else
  {
    test_module_->setDataFileName(save_path);
    test_count_ = test_count;
    total_test_time_ = ros::Duration(test_time);
  }

  current_job_index_ = 0;

  current_process_ = ON_START;

  return true;
}

bool TestManager::getPrevTestData(std::string &save_path, int &test_count, double &test_time)
{
  std::string file_name = test_module_->getDataFilePath() + robot_name_ + "_prev_test.yaml";

  // get prev test data
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(file_name.c_str());

    // get filename, count, time
    save_path = doc["save_file"].as<std::string>();
    test_count = doc["test_count"].as<int>();
    test_time = doc["test_time"].as<double>();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load prev test file.");
    return false;
  }

  return true;
}

void TestManager::savePrevTestData()
{
  if(test_count_ < 1)
  {
    ROS_ERROR("Wrong Test Count");
    return;
  }

  std::string file_name = test_module_->getDataFilePath() + robot_name_ + "_prev_test.yaml";

  std::string data_file_name = test_module_->getDataFileName();

  YAML::Emitter yaml_out;
  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "save_file" << YAML::Value << data_file_name;
  yaml_out << YAML::Key << "test_count" << YAML::Value << test_count_;
  yaml_out << YAML::Key << "test_time" << YAML::Value << total_test_time_.toSec();
  yaml_out << YAML::EndMap;

  std::ofstream fout(file_name.c_str());
  fout << yaml_out.c_str();  // dump it back into the file
  return;
}

void TestManager::setRobotName(const std::string& robot_name)
{
  robot_name_ = robot_name;

  if(test_module_ != NULL)
    test_module_->setRobotName(robot_name);
}

}
