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
  std::vector<std::string> data_list_;
  std::vector<double> data_value_;
};

#endif // JOINT_STATUS_H
