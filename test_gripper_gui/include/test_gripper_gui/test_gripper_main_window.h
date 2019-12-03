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

#ifndef TEST_GRIPPER_MAIN_WINDOW_H
#define TEST_GRIPPER_MAIN_WINDOW_H

#include <algorithm>

#include <QMainWindow>
#include <QRadioButton>
#include <QSignalMapper>
#include <QSettings>
#include <QString>
#include <QSizePolicy>

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
  void on_pushButton_lock_clicked(bool checked);

  void on_checkBox_set_stop_count_stateChanged(int state);

  void logToStatusBar(const std::string& message);

  void updateTestTime(const std::string &time);
  void updateTestCount(int count);
  void updateLoadcell(const std::string &state, double value);
  void clearSetEndTest();
  void setTestName(const std::string &test_name);

protected:
  void resizeEvent(QResizeEvent* event);

private:
//  void updateRobotUI();
//  void updatePositionUI();
  void resizeTitle();

  Ui::TestGripperMainWindow *ui_;
  QNodeTestGriper *q_node_;
  std::string title_;
};

}
#endif // TEST_GRIPPER_MAIN_WINDOW_H
