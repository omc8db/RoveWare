// RoveDynamixel.cpp
// Author: Gbenga Osibodu

#include "RoveDynamixel.h"

//BEGIN PUBLIC API

void RoveDynamixel_begin(RoveDynamixel* dyna, RoveDynamixelType type, uint8_t id, uint8_t uart_index, int baud) {
  dyna -> type = type;
  dyna -> id = id;
  dyna -> uart = RoveBoard_UART_open(uart_index, baud);
}

uint8_t RoveDynamixel_spinWheel(RoveDynamixel dyna, uint16_t speed) 
{
  return RoveDynamixel_setSpeed(RoveDynamixel dyna, uint16_t speed)
}

/*TODO GBENGA
uint8_t RoveDynamixel_rotateJoint(RoveDynamixel dyna, uint16_t angle, uint16_t speed) 
{
  return DYNAMIXEL_ERROR_UNKNOWN;
}

RoveDynamixel_readWheel(RoveDynamixel dyna, uint16_t* speed, uint16_t* load)
{
  return DYNAMIXEL_ERROR_UNKNOWN;
} 

RoveDynamixel_readJoint(RoveDynamixel dyna, uint16_t* angle, uint16_t* speed, uint16_t* load)
{
  return DYNAMIXEL_ERROR_UNKNOWN;
}

//better names for level? dyna_reply_delay? other important values?

RoveDynamixel_readConfig(RoveDynamixel dyna, uint16_t* max_torque, uint16_t* level, uint16_t* mode, uint8_t dyna_reply_delay )
{

}  

*/

//END PUBLIC API

void RoveDynamixel_sendPacket(RoveDynamixel dyna, uint8_t length, uint8_t* instruction) {
  int i;
  uint8_t checksum;
  
  checksum = dyna.id + (length + 1);
  for(i=0; i < length; i++) {
    checksum += instruction[i];
  }
  checksum = ~checksum;
  
  uint8_t packet[length + 5];
  
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = dyna.id;
  packet[3] = length + 1;
  memcpy(&(packet[4]), instruction, length);
  packet[length + 4] = checksum;
  
  RoveBoardUART_write(dyna.uart, packet, length + 5);
  wait(50);
  RoveBoardUART_read(dyna.uart, NULL, length + 5);
}

uint8_t RoveDynamixel_getReturnPacket(Dynamixel dyna, uint8_t* data, size_t data_size) {
  // To be fixed
  uint8_t id, length, error;
  uint8_t temp1, temp2;
  
  
  if(RoveBoard_UART_available(dyna.uart) == true){
    RoveBoard_UART_read(dyna.uart, &temp2, 1);
    
    while(RoveBoard_UART_available(dyna.uart) == true) { 
      temp1 = temp2;
      RoveBoard_UART_read(dyna.uart, &temp2, 1);
      if (temp1 == 255 && temp2 == 255) {
        RoveBoard_UART_read(dyna.uart, &id, 1);
        RoveBoard_UART_read(dyna.uart, &length, 1);
        RoveBoard_UART_read(dyna.uart, &error, 1);
        if (dataSize + 2 != length) {
          RoveBoard_UART_read(dyna.uart, NULL, length-2);
          return (error & DYNAMIXEL_ERROR_UNKNOWN);
        } else {
          RoveBoard_UART_read(dyna.uart, data, length-2);
          RoveBoard_UART_read(dyna.uart, NULL, 1);
          return error;
        }
      }
    }
  }
  return DYNAMIXEL_ERROR_UNKNOWN;
}

uint8_t RoveDynamixel_getError(RoveDynamixel dyna) {
  return RoveDynamixel_getReturnPacket(dyna, NULL, 0);
}

uint8_t RoveDynamixel_ping(RoveDynamixel dyna) {
  uint8_t msg_length = 1;
  uint8_t data = DYNAMIXEL_PING;
  
  RoveDynamixel_sendPacket(dyna, msgLength, &data);
  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

void RoveDynamixel_sendWriteCommand(RoveDynamixel dyna, uint8_t dynamixel_register, uint8_t data_length, uint8_t* data) {
  uint8_t buffer[dataLength + 2];

  buffer[0] = DYNAMIXEL_WRITE_DATA;
  buffer[1] = dynamixel_register;
  memcpy(&(buffer[2]), data, data_length);

  DynamixelSendPacket(dyna, dataLength + 2, buffer);
}

void Dynamixel_sendReadCommand(Dynamixel dyna, uint8_t dynamixel_register, uint8_t read_length) {
  uint8_t buffer[3];

  buffer[0] = DYNAMIXEL_READ_DATA;
  buffer[1] = dynamixel_register;
  buffer[2] = read_length;

  RoveDynamixel_sendPacket(dyna, 3, buffer);
}

uint8_t RoveDynamixel_setPosition(RoveDynamixel dyna, uint16_t position) 
{
  uint8_t msg_length = 2;
  uint8_t data[msg_length];
  
  //data[0] = DYNAMIXEL_WRITE_DATA;
  //data[1] = DYNAMIXEL_GOAL_POSITION_L;
  data[0] = position & 0x00FF;
  data[1] = position >> 8;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_GOAL_POSITION_L, msg_length, data);
  
  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setSpeed(RoveDynamixel dyna, uint16_t speed) 
{
  uint8_t msg_length = 2;
  uint8_t data[msg_length];
  
  data[0] = speed & 0x00FF;
  data[1] = speed >> 8;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_MOVING_SPEED_L, msg_length, data);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setId(RoveDynamixel* dyna, uint8_t id) 
{
  uint8_t msg_length = 1;
  
  RoveDynamixel_sendWriteCommand(*dyna, DYNAMIXEL_ID, msg_length, &id);
  
  dyna -> id = id;
  
  wait(TXDELAY);
  return RoveDynamixel_getError(*dyna);
}

uint8_t RoveDynamixel_setBaudRate(Dynamixel dyna, uint8_t baudByte) 
{
  uint8_t msg_length = 1;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_BAUD_RATE, msg_length, &baud_byte);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setReturnDelayTime(RoveDynamixel dyna, uint8_t return_delay_byte) 
{
  uint8_t msg_length = 1;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_RETURN_DELAY_TIME, msg_length, &return_delay_byte);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setMaxTorque(RoveDynamixel dyna, uint16_t max_torque) 
{
  uint8_t msg_length = 2;
  uint8_t data[msg_length];
  
  data[0] = maxTorque & 0x00FF;
  data[1] = maxTorque >> 8;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_MAX_TORQUE_L, msg_length, data);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setStatusReturnLevel(Dynamixel dyna, uint8_t level) 
{
  uint8_t msg_length = 1;
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_RETURN_LEVEL, msg_length, &level);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_setMode(RoveDynamixel dyna, RoveDynamixelMode mode) 
{
  uint8_t msg_length = 4;
  uint8_t data[msg_length];
  uint8_t ccw_high_byte;
  
  switch (mode) 
  {
    case Wheel:
      data[0] = 0x00;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      break;
    case Joint:
      if (dyna.type == MX) 
	  {
        ccw_high_byte = 0xFF & MX_HIGH_BYTE_MASK;
      }
      
      if (dyna.type == AX) 
	  {
        ccw_high_byte = 0xFF & AX_HIGH_BYTE_MASK;
      }
      
      data[0] = 0x00;
      data[1] = 0x00;
      data[2] = 0xFF;
      data[3] = ccw_high_byte;
      break;
    case MultiTurn:
      if (dyna.type == AX)
	  {
        return DYNAMIXEL_ERROR_UNKNOWN;
	  }
      
      if (dyna.type == MX) 
	  {
        ccw_high_byte = 0xFF & MX_HIGH_BYTE_MASK;
      }
      
      data[0] = 0xFF;
      data[1] = ccw_high_byte;
      data[2] = 0xFF;
      data[3] = ccw_high_byte;
      break;
    default:
      return DYNAMIXEL_ERROR_UNKNOWN;
  }
  
  RoveDynamixel_sendWriteCommand(dyna, DYNAMIXEL_CW_ANGLE_LIMIT_L, msg_length, data);

  wait(TXDELAY);
  return RoveDynamixel_getError(dyna);
}

uint8_t RoveDynamixel_getMode(RoveDynamixel dyna, RoveDynamixelMode* mode) 
{
  uint8_t msg_length = 3, data_size = 4, error;
  uint8_t data[msg_length], buffer[data_size];
  uint16_t cw_angle_limit, ccw_angle_limit;
  
  RoveDynamixel_sendReadCommand(dyna, DYNAMIXEL_CW_ANGLE_LIMIT_L, data_size);
  
  wait(TXDELAY);
  error = RoveDynamixel_getReturnPacket(dyna, buffer, data_size);
  
  cw_angle_limit = buffer[1];
  cw_angle_limit = (cw_angle_limit << 8) | buffer[0];
  ccw_angle_limit = buffer[3];
  ccw_angle_limit = (ccw_angle_limit << 8) | buffer[2];
  
  if (cw_angle_limit == 0 && ccw_angle_limit == 0) 
  {
    *mode = Wheel;
  }
  
  switch (dyna.type) {
    case AX:
      if (cw_angle_limitt == 0 && ccw_angle_limit == 0x03FF) 
	  {
        *mode = Joint;
      }
      break;
    case MX:
      if (cw_angle_limit == 0 && ccw_angle_limit == 0x0FFF) 
	  {
        *mode = Joint;
      }
      if (cw_angle_limit == 0x0FFF && ccw_angle_limit == 0x0FFF) 
	  {
        *mode = MultiTurn;
      }
  }
  return error;
}

uint8_t RoveDynamixel_getPresentPosition(Dynamixel dyna, uint16_t* pos) 
{
  uint8_t data_size = 2, error;
  uint8_t buffer[data_size];
  
  RoveDynamixel_sendReadCommand(dyna, DYNAMIXEL_PRESENT_POSITION_L, data_size);
  
  wait(TXDELAY);
  error = RoveDynamixel_getReturnPacket(dyna, buffer, data_size);
  
  *pos = buffer[1];
  *pos = (*pos << 8) | buffer[0];
  
  return  error;
}

uint8_t RoveDynamixel_getPresentSpeed(Dynamixel dyna, uint16_t* speed) 
{
  uint8_t data_size = 2, error;
  uint8_t buffer[data_size];
  
  RoveDynamixel_sendReadCommand(dyna, DYNAMIXEL_PRESENT_SPEED_L, data_size);
  
  wait(TXDELAY);
  error = RoveDynamixel_getReturnPacket(dyna, buffer, data_size);
  
  *speed = buffer[1];
  *speed = (*speed << 8) | buffer[0];
  
  return error;
}

uint8_t RoveDynamixel_getLoad(RoveDynamixel dyna, uint16_t* load) 
{
  uint8_t data_size = 2, error;
  uint8_t buffer[data_size];
  
  DynamixelSendReadCommand(dyna, DYNAMIXEL_PRESENT_LOAD_L, data_size);
  
  wait(TXDELAY);
  error = RoveDynamixel_getReturnPacket(dyna, buffer, data_size);
  
  *load = buffer[1];
  *load = (*load << 8) | buffer[0];
  
  return error;
}

uint8_t RoveDynamixel_getVoltage(RoveDynamixel dyna, uint8_t* voltage) 
{
  uint8_t data_size = 1;
  
  RoveDynamixel_sendReadCommand(dyna, DYNAMIXEL_PRESENT_VOLTAGE, data_size);
  
  wait(TXDELAY);
  return RoveDynamixel_getReturnPacket(dyna, voltage, data_size);
}

uint8_t RoveDynamixel_getTemperature(RoveDynamixel dyna, uint8_t* temp) 
{
  uint8_t data_size = 1;
  
  RoveDynamixel_sendReadCommand(dyna, DYNAMIXEL_PRESENT_TEMPERATURE, data_size);

  wait(TXDELAY);
  return RoveDynamixel_getReturnPacket(dyna, temp, data_size);
}
