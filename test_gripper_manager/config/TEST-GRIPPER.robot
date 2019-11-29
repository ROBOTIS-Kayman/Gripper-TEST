[ port info ]
# PORT NAME     | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0    | 2000000  | joint_1
#/dev/U2D2       | 2000000  | joint_1

[ device info ]
# TYPE    | PORT NAME       | ID  | MODEL              | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0    | 1   | H54-200-S500-R(A)  | 2.0      | joint_1      | hardware_error_status, goal_position, present_current, present_position, present_temperature
dynamixel | /dev/ttyUSB0    | 2   | H54-100-S500-R(A)  | 2.0      | joint_2      | hardware_error_status, goal_position, present_current, present_position, present_temperature
dynamixel | /dev/ttyUSB0    | 3   | RH-P12-RN(A)       | 2.0      | gripper      | hardware_error_status, goal_position, present_current, present_position, present_temperature
sensor    | /dev/ttyUSB0    | 4   | POWERXEL           | 2.0      | powerxel     | voltage, current
