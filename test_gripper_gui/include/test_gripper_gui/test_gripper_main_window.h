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

#ifndef TEST_GRIPPER_MAIN_WINDOW_H
#define TEST_GRIPPER_MAIN_WINDOW_H

#include <QMainWindow>
#include <QRadioButton>
#include <QSignalMapper>
#include <QSettings>
#include <QString>

#include "ui_test_gripper_main_window.h"
#include "q_ros_node.h"

Q_DECLARE_METATYPE(std::string)

namespace Ui {
class TestGripperMainWindow;
}

namespace test_gripper_gui {

class TestGripperMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit TestGripperMainWindow(int argc, char **argv, QWidget *parent = 0);
  ~TestGripperMainWindow();

  void readSettings();  // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function

public Q_SLOTS:
 /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_pushButton_ready_clicked(bool clicked);
  void on_pushButton_resume_clicked(bool clicked);
  void on_pushButton_start_clicked(bool clicked);
  void on_pushButton_start_continue_clicked(bool clicked);
  void on_pushButton_stop_clicked(bool clicked);
  void on_pushButton_e_stop_clicked(bool clicked);

  void logToStatusBar(const std::string& message);

  void updateTestTime(const std::string &time);
  void updateTestCount(int count);

private:
//  void updateRobotUI();
//  void updatePositionUI();

  Ui::TestGripperMainWindow *ui_;
  QNodeTestGriper *q_node_;
};

}
#endif // TEST_GRIPPER_MAIN_WINDOW_H
