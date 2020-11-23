#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>

#include <motion_udp/input.h>
#include <motion_udp/viz_types.h>
#include <motion_udp/udpserver.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <motion_udp/ContactData.h>
#include <motion_udp/ImuData.h>
#include <motion_udp/JointData.h>
#include <motion_udp/JointType.h>
#include <motion_udp/MessageData.h>
#include <motion_udp/RobotJointData.h>
#include <motion_udp/udpMessage.h>

using namespace std;

boost::shared_ptr<InputSocket> input_;
UDP_DATA udp_data;

ros::Publisher udp_pub;
ros::Publisher joint_data_pub;
ros::Publisher imu_data_pub;
ros::Publisher contact_data_pub;

boost::thread* recvudp_thread_;
int packet_size_ = 0;

void recvudp()
{
  uint8_t tmp_packet[packet_size_];
  UDP_DATA temp_data;
  uint32_t crc_buff[packet_size_/4];
  uint32_t crc=0;

  while(true)
  {
    int rc = input_->getPacket(tmp_packet, packet_size_);

    if (rc == 0) break;       // got a full packet?
    if (rc < 0) ROS_ERROR("getPacket Error!"); // end of file reached?
  }

  memcpy(temp_data.buff,tmp_packet,packet_size_);

//  cout<<"msg:"<<sizeof(Message)<<endl;
//  cout<<"rj1:"<<sizeof(RobotJointData)<<endl;
//  cout<<"rj2:"<<sizeof(ImuData)<<endl;
//  cout<<"rj3:"<<sizeof(ContactData)<<endl;
//  cout<<"rj4:"<<sizeof(MessageData)<<endl;
  memcpy(crc_buff,temp_data.buff,packet_size_/4);

  for(int i ;i < packet_size_/4-1 ; i++)
  {
    crc += crc_buff[i];
  }
//  if(crc == temp_data.udp_msg.check_sum)
//  {
    udp_data = temp_data;
//  }
}
void udpmsg_publish()
{
  motion_udp::udpMessage UDP_MSG;
  UDP_MSG.code = udp_data.udp_msg.code;
  UDP_MSG.size = udp_data.udp_msg.size;
  UDP_MSG.count = udp_data.udp_msg.count;
  UDP_MSG.message_data.timestamp = udp_data.udp_msg.message_data.timestamp;
  //触地判断
  UDP_MSG.message_data.contact_data.fl = udp_data.udp_msg.message_data.contact_data.fl;
  UDP_MSG.message_data.contact_data.fr = udp_data.udp_msg.message_data.contact_data.fr;
  UDP_MSG.message_data.contact_data.hl = udp_data.udp_msg.message_data.contact_data.hl;
  UDP_MSG.message_data.contact_data.hr = udp_data.udp_msg.message_data.contact_data.hr;
  //imu信息
  UDP_MSG.message_data.imu_data.acc_x = udp_data.udp_msg.message_data.imu_data.acc_x;
  UDP_MSG.message_data.imu_data.acc_y = udp_data.udp_msg.message_data.imu_data.acc_y;
  UDP_MSG.message_data.imu_data.acc_z = udp_data.udp_msg.message_data.imu_data.acc_z;
  //
  UDP_MSG.message_data.imu_data.angle_pitch = udp_data.udp_msg.message_data.imu_data.angle_pitch;
  UDP_MSG.message_data.imu_data.angle_yaw = udp_data.udp_msg.message_data.imu_data.angle_yaw;
  UDP_MSG.message_data.imu_data.angle_roll = udp_data.udp_msg.message_data.imu_data.angle_roll;
  //
  UDP_MSG.message_data.imu_data.angular_velocity_pitch = udp_data.udp_msg.message_data.imu_data.angular_velocity_pitch;
  UDP_MSG.message_data.imu_data.angular_velocity_yaw = udp_data.udp_msg.message_data.imu_data.angular_velocity_yaw;
  UDP_MSG.message_data.imu_data.angular_velocity_roll = udp_data.udp_msg.message_data.imu_data.angular_velocity_roll;
  //电机位置信息
  UDP_MSG.message_data.robot_joint_data.position.fl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.position.fl.hip_x;
  UDP_MSG.message_data.robot_joint_data.position.fl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.position.fl.hip_y;
  UDP_MSG.message_data.robot_joint_data.position.fl.knee = udp_data.udp_msg.message_data.robot_joint_data.position.fl.knee;

  UDP_MSG.message_data.robot_joint_data.position.fr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.position.fr.hip_x;
  UDP_MSG.message_data.robot_joint_data.position.fr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.position.fr.hip_y;
  UDP_MSG.message_data.robot_joint_data.position.fr.knee = udp_data.udp_msg.message_data.robot_joint_data.position.fr.knee;

  UDP_MSG.message_data.robot_joint_data.position.hl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.position.hl.hip_x;
  UDP_MSG.message_data.robot_joint_data.position.hl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.position.hl.hip_y;
  UDP_MSG.message_data.robot_joint_data.position.hl.knee = udp_data.udp_msg.message_data.robot_joint_data.position.hl.knee;

  UDP_MSG.message_data.robot_joint_data.position.hr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.position.hr.hip_x;
  UDP_MSG.message_data.robot_joint_data.position.hr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.position.hr.hip_y;
  UDP_MSG.message_data.robot_joint_data.position.hr.knee = udp_data.udp_msg.message_data.robot_joint_data.position.hr.knee;
  //电机速度信息
  UDP_MSG.message_data.robot_joint_data.velocity.fl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.velocity.fl.hip_x;
  UDP_MSG.message_data.robot_joint_data.velocity.fl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.velocity.fl.hip_y;
  UDP_MSG.message_data.robot_joint_data.velocity.fl.knee = udp_data.udp_msg.message_data.robot_joint_data.velocity.fl.knee;

  UDP_MSG.message_data.robot_joint_data.velocity.fr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.velocity.fr.hip_x;
  UDP_MSG.message_data.robot_joint_data.velocity.fr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.velocity.fr.hip_y;
  UDP_MSG.message_data.robot_joint_data.velocity.fr.knee = udp_data.udp_msg.message_data.robot_joint_data.velocity.fr.knee;

  UDP_MSG.message_data.robot_joint_data.velocity.hl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.velocity.hl.hip_x;
  UDP_MSG.message_data.robot_joint_data.velocity.hl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.velocity.hl.hip_y;
  UDP_MSG.message_data.robot_joint_data.velocity.hl.knee = udp_data.udp_msg.message_data.robot_joint_data.velocity.hl.knee;

  UDP_MSG.message_data.robot_joint_data.velocity.hr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.velocity.hr.hip_x;
  UDP_MSG.message_data.robot_joint_data.velocity.hr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.velocity.hr.hip_y;
  UDP_MSG.message_data.robot_joint_data.velocity.hr.knee = udp_data.udp_msg.message_data.robot_joint_data.velocity.hr.knee;

  //电机力矩信息
  UDP_MSG.message_data.robot_joint_data.torque.fl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.torque.fl.hip_x;
  UDP_MSG.message_data.robot_joint_data.torque.fl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.torque.fl.hip_y;
  UDP_MSG.message_data.robot_joint_data.torque.fl.knee = udp_data.udp_msg.message_data.robot_joint_data.torque.fl.knee;

  UDP_MSG.message_data.robot_joint_data.torque.fr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.torque.fr.hip_x;
  UDP_MSG.message_data.robot_joint_data.torque.fr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.torque.fr.hip_y;
  UDP_MSG.message_data.robot_joint_data.torque.fr.knee = udp_data.udp_msg.message_data.robot_joint_data.torque.fr.knee;

  UDP_MSG.message_data.robot_joint_data.torque.hl.hip_x = udp_data.udp_msg.message_data.robot_joint_data.torque.hl.hip_x;
  UDP_MSG.message_data.robot_joint_data.torque.hl.hip_y = udp_data.udp_msg.message_data.robot_joint_data.torque.hl.hip_y;
  UDP_MSG.message_data.robot_joint_data.torque.hl.knee = udp_data.udp_msg.message_data.robot_joint_data.torque.hl.knee;

  UDP_MSG.message_data.robot_joint_data.torque.hr.hip_x = udp_data.udp_msg.message_data.robot_joint_data.torque.hr.hip_x;
  UDP_MSG.message_data.robot_joint_data.torque.hr.hip_y = udp_data.udp_msg.message_data.robot_joint_data.torque.hr.hip_y;
  UDP_MSG.message_data.robot_joint_data.torque.hr.knee = udp_data.udp_msg.message_data.robot_joint_data.torque.hr.knee;

  UDP_MSG.check_sum = udp_data.udp_msg.check_sum;
  udp_pub.publish(UDP_MSG);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_udpServer");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  udp_pub = n.advertise<motion_udp::udpMessage>("udp_msg",10);

  joint_data_pub = n.advertise<motion_udp::RobotJointData>("joint_data",10);
  imu_data_pub = n.advertise<motion_udp::ImuData>("imu_data",10);
  contact_data_pub = n.advertise<motion_udp::ContactData>("contact_data",10);

  private_nh.param<int>("packet_size", packet_size_, 208);


  // read data from live socket
  input_.reset(new InputSocket(private_nh));

    // 开启一个线程接受UDP数据
//  recvudp_thread_ = new boost::thread(&recvUDPThread);
//  recvudp_thread_->join();

  ros::Rate rate(100);
  /* 开始send数据 */
  while(ros::ok())
  {
    recvudp();
    udpmsg_publish();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
