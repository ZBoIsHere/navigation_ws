#ifndef UDPSERVER_H
#define UDPSERVER_H
#include "viz_types.h"

union UDP_DATA
{
  char buff[208];//208
  Message udp_msg;
};

#endif // UDPSERVER_H
