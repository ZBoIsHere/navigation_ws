///////////////////////////
/// @file viz_types.h
/// 
/// @time 2019-11-15
/// @author moxiaobo0571@163.com
///////////////////////////

#ifndef VIZ_TYPES_H_
#define VIZ_TYPES_H_
#include <inttypes.h>

#pragma pack(4)  //4byte
/// 所有类型都采用小端（LE）字节序.

/// @todo 数据范围
/// @todo 数据精度

/// 机器人中的关节类型
struct JointType{
  float hip_x,hip_y,knee;
};
/// 所有关节位置或速度的关节数据类型
/// front hip left right
struct JointData{
  JointType fl,fr,hl,hr;
};

/// 机器人关节数据  4*12*3 = 144 bytes
struct RobotJointData{
  JointData position,velocity,torque;
};          //rad     rad/s   Nm

/// 陀螺仪数据 4*9 = 36 bytes
struct ImuData{
  float angle_roll,angle_pitch,angle_yaw;//deg
  float angular_velocity_roll,angular_velocity_pitch,angular_velocity_yaw;//rad/s
  float acc_x,acc_y,acc_z;//m/s^2
};


/// 触地数据 0表示悬空，1表示触地 1*4 = 4 bytes
struct ContactData{
  uint32_t fl :8;
  uint32_t fr :8;
  uint32_t hl :8;
  uint32_t hr :8;
};

/// 发送报文的主体数据结构
struct MessageData{
  double timestamp;//s
  RobotJointData robot_joint_data;
  ImuData imu_data;
  ContactData contact_data;
};

/// 报文完整数据结构
struct Message{
  uint32_t code;  ///< 预定义为 0x0000000E
  uint32_t size;  ///< 值为 sizeof(struct Message);
  uint32_t count; ///< 循环计数

  MessageData message_data;

  uint32_t check_sum;   ///< 除此之外的数据以32位累加和校验值
};

#endif /* VIZ_TYPES_H_ */
