#include <arpa/inet.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h> /* See NOTES */

#include <iostream>
using namespace std;

// udp相关参数/*
#define SOCKET_PORT 43897  // 定义socket绑定的端口
#define LISTEN_BACKLOG 10  // 设置服务器最大的监听数量
#define ARRAY_LENGTH 256   // 定义缓冲区的大小
struct Cmd {
  uint32_t code;
  uint32_t value;
  uint32_t type;
  char buffer[255];
} cmd;

int main(int argc, char **argv) {
  ros::init(argc, argv, "udpThread");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("udpThread", 10);
  ros::Rate loop_rate(10);

  int iSocketServerFd;  // 定义服务器端的socket文件描述符
  int iSocketClientFd;  // 定义客服端的socket文件描述符

  struct sockaddr_in tSocketServerAddr;
  struct sockaddr_in tSocketClientAddr;

  unsigned int iSockAddrLen;
  int iRet;

  int iRecvLen;

  // signal(SIGCHLD,SIG_IGN);

  /* 1、打开一个socket */
  iSocketServerFd =
      socket(AF_INET, SOCK_DGRAM, 0);  // 网路类型为ipv4，链接类型为udp
  if (iSocketServerFd == -1) {
    printf("socket error!\n");
  }

  /* 2、绑定 */
  /* 2.1 设置要绑定的服务器端 */
  tSocketServerAddr.sin_family = AF_INET;           // 网络类型为ipv4
  tSocketServerAddr.sin_port = htons(SOCKET_PORT);  // 设置服务器端口
  tSocketServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  memset(tSocketServerAddr.sin_zero, 0, 8);

  iSockAddrLen = sizeof(struct sockaddr);

  /* 2.2 对服务器端进行绑定 */
  iRet = bind(iSocketServerFd, (const struct sockaddr *)&tSocketServerAddr,
              iSockAddrLen);
  if (iRet == -1) {
    printf("bind error!\n");
  }

  while (ros::ok()) {
    cmd.code = 0;
    /* 3、接收数据 */
    std::cout << "waiting for cmd" << endl;
    iRecvLen = recvfrom(iSocketServerFd, &cmd, ARRAY_LENGTH, 0,
                        (struct sockaddr *)&tSocketServerAddr, &iSockAddrLen);
    if (iRecvLen <= 0) {
    } else {
      std::cout << "Receive a message from : "
                << inet_ntoa(tSocketClientAddr.sin_addr) << endl;
      std::cout << "The message is : " << cmd.code
                << endl;  // cmd.code 为收到机器人反馈的值
    }

    if (cmd.code == 1 || cmd.code == 2)  //
    {
      std_msgs::Int32 cmd_msg;
      cmd_msg.data = cmd.code;
      pub.publish(cmd_msg);
      cmd.code = 0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  close(iSocketServerFd);
}