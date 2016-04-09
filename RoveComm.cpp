// RoveComm.c
// Author: Gbenga Osibodu

#include "RoveComm.h"
#include <string.h>

#define ROVECOMM_VERSION 1
#define ROVECOMM_HEADER_LENGTH 8
#define ROVECOMM_PORT 11000

#define UDP_TX_PACKET_MAX_SIZE 1500
#define ROVECOMM_MAX_SUBSCRIBERS 5

#define ROVECOMM_ACKNOWLEDGE_FLAG   1

#define ROVECOMM_PING               0x0001
#define ROVECOMM_PING_REPLY         0x0002
#define ROVECOMM_SUBSCRIBE          0x0003
#define ROVECOMM_UNSUBSCRIBE        0x0004
#define ROVECOMM_FORCE_UNSUBSCRIBE  0x0005
#define ROVECOMM_ACKNOWLEDGE_MSG    0x0006


uint8_t RoveCommBuffer[UDP_TX_PACKET_MAX_SIZE];
RoveIP RoveCommSubscribers[ROVECOMM_MAX_SUBSCRIBERS]; 

void RoveComm_sendMsgTo(uint16_t data_id, size_t size, const void* data, uint16_t seq_num, uint8_t flags, RoveIP DestIP, uint16_t dest_port);
static void RoveComm_parseMsg(uint8_t* buffer, uint16_t* data_id, size_t* size, void* data, uint16_t* seq_num, uint8_t* flags);
static void RoveComm_handleSystemMsg(uint8_t* buffer, uint16_t* dataID, size_t* size, void* data, uint16_t* seq_num, uint8_t* flags, RoveIP IP);
static bool RoveComm_addSubscriber(RoveIP IP);

void RoveComm_begin(uint8_t ip_octet1, uint8_t ip_octet2, uint8_t ip_octet3, uint8_t ip_octet4) 
{
  RoveIP IP = RoveEthernet_setIP(ip_octet1, ip_octet2, ip_octet3, ip_octet4);
  
  RoveEthernet_networkingStart(IP);
  
  RoveEthernet_udpSocketListen(ROVECOMM_PORT);
  
  int i;
  for (i=0; i < ROVECOMM_MAX_SUBSCRIBERS; i++) 
  {
    RoveCommSubscribers[i] = ROVE_IP_ADDR_NONE;
  }
}

void RoveComm_read(uint16_t* data_id, size_t* size, void* data) 
{
  uint8_t flags = 0;
  uint16_t seq_num = 0;
  RoveIP SenderIP;
  
  *data_id = 0;
  *size = 0;
  
  if (RoveEthernet_getUdpMsg(&SenderIP, RoveCommBuffer, sizeof(RoveCommBuffer)) == ROVE_ETHERNET_ERROR_SUCCESS) 
  {
    RoveComm_parseMsg(RoveCommBuffer, data_id, size, data, &seq_num, &flags);  
    RoveComm_handleSystemMsg(RoveCommBuffer, data_id, size, data, &seq_num, &flags, Sender_ip);
  }
}

static void RoveComm_parseMsg(uint8_t* buffer, uint16_t* data_id, size_t* size, void* data, uint16_t* seq_num, uint8_t* flags) 
{
  int protocol_version = buffer[0];
  switch (protocol_version) 
  {
    case 1:
      *seq_num = buffer[1];
      *seq_num = (*seq_num << 8) | buffer[2];
      *flags = buffer[3];
      *data_id = buffer[4];
      *data_id = (*data_id << 8) | buffer[5];
      *size = buffer[6];
      *size = (*size << 8) | buffer[7];
      
      memcpy(data, &(buffer[8]), *size);
  }
}

void RoveComm_SendMsgTo(uint16_t data_id, size_t size, const void* data, uint16_t seq_num, uint8_t flags, RoveIP DestIP, uint16_t dest_port) 
{
  size_t packet_size = size + ROVECOMM_HEADER_LENGTH;
  uint8_t buffer[packet_size];
  
  buffer[0] = ROVECOMM_VERSION;
  buffer[1] = seq_num >> 8;
  buffer[2] = seq_num & 0x00FF;
  buffer[3] = flags;
  buffer[4] = data_id >> 8;
  buffer[5] = data_id & 0x00FF;
  buffer[6] = size >> 8;
  buffer[7] = size & 0x00FF;
  
  memcpy(&(buffer[8]), data, size);

  RoveEthernet_sendUdpPacket(DestIP, dest_port, buffer, packet_size);
}

void RoveComm_write(uint16_t data_id, size_t size, const void* data) 
{
  int i = 0; 
  
  for (i=0; i < ROVECOMM_MAX_SUBSCRIBERS; i++) 
  {
    if (!(RoveCommSubscribers[i] == ROVE_IP_ADDR_NONE)) 
    {
      RoveComm_sendMsgTo(data_id, size, data, 0x00FF, 0, RoveCommSubscribers[i], ROVECOMM_PORT);
    }
  }
}

static bool RoveComm_addSubscriber(RoveIP IP) 
{
  int i = 0;

  for (i=0; i<ROVECOMM_MAX_SUBSCRIBERS; i++) 
  {
    if (RoveCommSubscribers[i] == IP) 
    {
      return true;
    }
    if (RoveCommSubscribers[i] == ROVE_IP_ADDR_NONE) 
    {
      RoveCommSubscribers[i] = IP;
      return true;
    }
  }
  
  return false;
}

static void RoveComm_handleSystemMsg(uint8_t* buffer, uint16_t* data_id, size_t* size, void* data, uint16_t* seq_num, uint8_t* flags, RoveIP IP) 
{
  if (*flags & ROVECOMM_ACKNOWLEDGE_FLAG != 0) 
  {
    RoveComm_sendMsgTo(ROVECOMM_ACKNOWLEDGE_MSG, sizeof(uint16_t), data_id, 0x00FF, 0, IP, ROVECOMM_PORT);
  }

  switch (*data_id) 
  {
    case ROVECOMM_PING:
      RoveComm_sendMsgTo(ROVECOMM_PING_REPLY, sizeof(uint16_t), seq_num, 0x00FF, 0, IP, ROVECOMM_PORT);
      break;
    case ROVECOMM_PING_REPLY:
      break;
    case ROVECOMM_SUBSCRIBE:
      RoveComm_addSubscriber(IP);
      break;
    case ROVECOMM_ACKNOWLEDGE_MSG:
      break;
    default:
      return;
  }
  *data_id = 0;
  *size = 0;
}

