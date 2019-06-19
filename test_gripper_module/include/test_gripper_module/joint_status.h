#ifndef JOINT_STATUS_H
#define JOINT_STATUS_H

#include <vector>
#include <string>

class JointStatus
{
public:
  JointStatus();
  JointStatus(std::string joint_name);

  std::string joint_name_;
  std::string joint_status_;
  double base_current_;
  std::string check_current_task_;   // task_1 : start, task_2 : end
  std::vector<std::string> data_list_;
  std::vector<double> data_value_;
};

#endif // JOINT_STATUS_H
