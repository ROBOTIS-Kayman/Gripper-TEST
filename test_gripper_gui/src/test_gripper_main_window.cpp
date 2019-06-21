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

#include "../include/test_gripper_gui/test_gripper_main_window.h"

namespace test_gripper_gui {

TestGripperMainWindow::TestGripperMainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::TestGripperMainWindow)
{
  ui_->setupUi(this);

  q_node_ = new QNodeTestGriper(argc, argv);
  q_node_->init();

  qRegisterMetaType<std::string>();
  QObject::connect(q_node_, SIGNAL(shutdown_ros()), this, SLOT(close()));
  QObject::connect(q_node_, SIGNAL(log(std::string)), this, SLOT(logToStatusBar(std::string)));
  QObject::connect(q_node_, SIGNAL(updateTestTime(std::string)), this, SLOT(updateTestTime(std::string)));
  QObject::connect(q_node_, SIGNAL(updateTestCount(int)), this, SLOT(updateTestCount(int)));

  readSettings();

  // init display
  updateTestTime("0000:00:00");
  updateTestCount(0);
}

TestGripperMainWindow::~TestGripperMainWindow()
{
  delete ui_;
}

void TestGripperMainWindow::on_pushButton_ready_clicked(bool clicked)
{
  q_node_->sendCommand("ready");
}

void TestGripperMainWindow::on_pushButton_resume_clicked(bool clicked)
{
  q_node_->sendCommand("resume");
}

void TestGripperMainWindow::on_pushButton_start_clicked(bool clicked)
{
  q_node_->sendCommand("start");
}

void TestGripperMainWindow::on_pushButton_start_continue_clicked(bool clicked)
{
  q_node_->sendCommand("start_continue");
}

void TestGripperMainWindow::on_pushButton_stop_clicked(bool clicked)
{
  q_node_->sendCommand("stop_end");
}

void TestGripperMainWindow::on_pushButton_e_stop_clicked(bool clicked)
{
  q_node_->sendCommand("stop");
}

void TestGripperMainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "test_gripper_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void TestGripperMainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "test_gripper_gui");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void TestGripperMainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

void TestGripperMainWindow::updateTestTime(const std::string &time)
{
  QString q_string_time = QString::fromStdString(time);
//  QString q_string_time("0000:00:00");

  ui_->lcdNumber_test_time->display(q_string_time);
}

void TestGripperMainWindow::updateTestCount(int count)
{
  QString q_string_count = QString::number(count);

  ui_->lcdNumber_test_count->display(q_string_count);
}

void TestGripperMainWindow::logToStatusBar(const std::string& message)
{
  QString q_message = QString::fromStdString(message);
  ui_->statusbar->showMessage(q_message);
}

}
