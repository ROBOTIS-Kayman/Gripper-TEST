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

#include "../include/test_master_gui/test_master_main_window.h"

namespace test_master_gui {

TestMasterMainWindow::TestMasterMainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::TestMasterMainWindow)
{
  ui_->setupUi(this);

  q_node_ = new QNodeTestMaster(argc, argv);
  q_node_->init();

  qRegisterMetaType<std::string>();
  QObject::connect(q_node_, SIGNAL(shutdown_ros()), this, SLOT(close()));
  QObject::connect(q_node_, SIGNAL(log(std::string)), this, SLOT(logToStatusBar(std::string)));
  QObject::connect(q_node_, SIGNAL(updateTestTime(std::string)), this, SLOT(updateTestTime(std::string)));
  QObject::connect(q_node_, SIGNAL(updateTestCount(int)), this, SLOT(updateTestCount(int)));
//  QObject::connect(q_node_, SIGNAL(clearSetEndTest()), this, SLOT(clearSetEndTest()));
//  QObject::connect(q_node_, SIGNAL(updateLoadcell(std::string, double)), this, SLOT(updateLoadcell(std::string, double)));
  readSettings();

  // init display
  updateTestTime("0000:00:00");
  updateTestCount(0);
  std::string robot_name;
  if(q_node_->getRobotName(robot_name))
    setTestName("RH-P12-RN Test(" + robot_name + ")");

  resizeTitle();

  // load robot list
  std::vector<std::string> robot_list;
  q_node_->getRobotList(robot_list);

  for(auto r_it = robot_list.begin(); r_it != robot_list.end(); ++r_it)
  {
    QString item = tr((*r_it).c_str());
    ui_->comboBox_robot->addItem(item);
  }
}

TestMasterMainWindow::~TestMasterMainWindow()
{
  delete ui_;
}

void TestMasterMainWindow::on_pushButton_ready_clicked(bool clicked)
{
  q_node_->sendCommandToAll("ready");
}

void TestMasterMainWindow::on_pushButton_resume_clicked(bool clicked)
{
  q_node_->sendCommandToAll("resume");
}

//void TestMasterMainWindow::on_pushButton_start_clicked(bool clicked)
//{
//  q_node_->sendCommandToAll("start");
//}

void TestMasterMainWindow::on_pushButton_start_continue_clicked(bool clicked)
{
  q_node_->sendCommandToAll("start_continue");
}

void TestMasterMainWindow::on_pushButton_stop_clicked(bool clicked)
{
  q_node_->sendCommandToAll("stop_end");
}

void TestMasterMainWindow::on_pushButton_e_stop_clicked(bool clicked)
{
  q_node_->sendCommandToAll("stop");
}

//void TestMasterMainWindow::on_pushButton_lock_clicked(bool checked)
//{
//  QList<QAbstractButton*> buttons = ui_->groupBox_control->findChildren<QAbstractButton *>();

//  foreach(QAbstractButton* button, buttons)
//  {
//    button->setEnabled(!checked);
//  }
//}

void TestMasterMainWindow::on_pushButton_loadcell_command_clicked(bool checked)
{
  std::string robot = ui_->comboBox_robot->currentText().toStdString();
  std::string command = ui_->comboBox_command->currentText().toStdString();

  q_node_->sendCommand(robot, command);
}

//void TestMasterMainWindow::on_checkBox_set_stop_count_stateChanged(int state)
//{
//  if(state == Qt::Unchecked)
//  {
//    ui_->spinBox_stop_count->setReadOnly(false);
//    q_node_->setEndCount(false, 0);
//  }
//  else if(state == Qt::Checked)
//  {
//    ui_->spinBox_stop_count->setReadOnly(true);
//    q_node_->setEndCount(true, ui_->spinBox_stop_count->value());
//  }
//}

void TestMasterMainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "test_master_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void TestMasterMainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "test_master_gui");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void TestMasterMainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

//void TestMasterMainWindow::updateLoadcell(const std::string &state, double value)
//{
//  ui_->label_loadcell->setText(QString::fromStdString(state));
//  ui_->spinBox_loadcell->setValue(value);
//}

void TestMasterMainWindow::updateTestTime(const std::string &time)
{
  QString q_string_time = QString::fromStdString(time);
  //  QString q_string_time("0000:00:00");

  ui_->lcdNumber_test_time->display(q_string_time);
}

void TestMasterMainWindow::updateTestCount(int count)
{
  QString q_string_count = QString::number(count);

  ui_->lcdNumber_test_count->display(q_string_count);
}

//void TestMasterMainWindow::clearSetEndTest()
//{
//  ui_->checkBox_set_stop_count->click();
//}

void TestMasterMainWindow::setTestName(const std::string &test_name)
{
  ui_->label_title->setText(QString::fromStdString(test_name));
}

void TestMasterMainWindow::logToStatusBar(const std::string& message)
{
  QString q_message = QString::fromStdString(message);
  ui_->statusbar->showMessage(q_message);
}

void TestMasterMainWindow::resizeEvent(QResizeEvent* event)
{
  QMainWindow::resizeEvent(event);

  resizeTitle();
}

void TestMasterMainWindow::resizeTitle()
{
  int font_size = std::min<int>(ui_->label_title->geometry().height() * 0.6, ui_->label_title->geometry().width() * 0.08);
  font_size = std::max<int>(50, font_size);
  QFont font =  ui_->label_title->font();
  font.setPixelSize(font_size);
  ui_->label_title->setFont(font);
}


}

