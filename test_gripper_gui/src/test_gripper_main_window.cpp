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
