// RoveDynamixel.h
// Author: Gbenga Osibodu

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "../RoveBoard/RoveBoard.h"

// DYNAMIXEL EEPROM AREA
#define DYNAMIXEL_MODEL_NUMBER_L           0
#define DYNAMIXEL_MODEL_NUMBER_H           1
#define DYNAMIXEL_VERSION                  2
#define DYNAMIXEL_ID                       3
#define DYNAMIXEL_BAUD_RATE                4
#define DYNAMIXEL_RETURN_DELAY_TIME        5
#define DYNAMIXEL_CW_ANGLE_LIMIT_L         6
#define DYNAMIXEL_CW_ANGLE_LIMIT_H         7
#define DYNAMIXEL_CCW_ANGLE_LIMIT_L        8
#define DYNAMIXEL_CCW_ANGLE_LIMIT_H        9
#define DYNAMIXEL_LIMIT_TEMPERATURE        11
#define DYNAMIXEL_DOWN_LIMIT_VOLTAGE       12
#define DYNAMIXEL_UP_LIMIT_VOLTAGE         13
#define DYNAMIXEL_MAX_TORQUE_L             14
#define DYNAMIXEL_MAX_TORQUE_H             15
#define DYNAMIXEL_RETURN_LEVEL             16
#define DYNAMIXEL_ALARM_LED                17
#define DYNAMIXEL_ALARM_SHUTDOWN           18

// MX SERIES EEPROM
#define MX_MULTI_TURN_OFFSET_L             20
#define MX_MULTI_TURN_OFFSET_H             21
#define MX_RESOLUTION_DIVIDER              22

// DYNAMIXEL RAM AREA
#define DYNAMIXEL_TORQUE_ENABLE            24
#define DYNAMIXEL_LED                      25
#define DYNAMIXEL_GOAL_POSITION_L          30
#define DYNAMIXEL_GOAL_POSITION_H          31
#define DYNAMIXEL_MOVING_SPEED_L           32
#define DYNAMIXEL_MOVING_SPEED_H           33
#define DYNAMIXEL_TORQUE_LIMIT_L           34
#define DYNAMIXEL_TORQUE_LIMIT_H           35
#define DYNAMIXEL_PRESENT_POSITION_L       36
#define DYNAMIXEL_PRESENT_POSITION_H       37
#define DYNAMIXEL_PRESENT_SPEED_L          38
#define DYNAMIXEL_PRESENT_SPEED_H          39
#define DYNAMIXEL_PRESENT_LOAD_L           40
#define DYNAMIXEL_PRESENT_LOAD_H           41
#define DYNAMIXEL_PRESENT_VOLTAGE          42
#define DYNAMIXEL_PRESENT_TEMPERATURE      43
#define DYNAMIXEL_REGISTERED_INSTRUCTION   44
#define DYNAMIXEL_MOVING                   46
#define DYNAMIXEL_LOCK                     47
#define DYNAMIXEL_PUNCH_L                  48
#define DYNAMIXEL_PUNCH_H                  49

// AX SERIES RAM
#define AX_CW_COMPLIANCE_MARGIN            26
#define AX_CCW_COMPLIANCE_MARGIN           27
#define AX_CW_COMPLIANCE_SLOPE             28
#define AX_CCW_COMPLIANCE_SLOPE            29

// MX SERIES RAM
#define MX_D_GAIN                          26
#define MX_I_GAIN                          27
#define MX_P_GAIN                          28
#define MX_GOAL_ACCELERATION               73

// Instructions
#define DYNAMIXEL_PING                     1
#define DYNAMIXEL_READ_DATA                2
#define DYNAMIXEL_WRITE_DATA               3
#define DYNAMIXEL_REG_WRITE                4
#define DYNAMIXEL_ACTION                   5
#define DYNAMIXEL_RESET                    6

#define MX_HIGH_BYTE_MASK                  0x0F
#define AX_HIGH_BYTE_MASK                  0x03

#define TXDELAY 2000

typedef enum 
{
  AX,
  MX
} RoveDynamixelType;

typedef enum 
{
  Wheel = 0,
  Joint = 1,
  MultiTurn = 2
} RoveDynamixelMode;

typedef struct 
{
  uint8_t id;
  RoveDynamixelType type;
  RoveUART_Handle uart;
} RoveDynamixel;

typedef struct RoveDynamixel *RoveDynamixelHandle;

typedef enum 
{
  DYNAMIXEL_ERROR_SUCCESS = 0,
  DYNAMIXEL_ERROR_VOLTAGE = 1,
  DYNAMIXEL_ERROR_ANGLE_LIMIT = 2,
  DYNAMIXEL_ERROR_OVERHEATING = 4,
  DYNAMIXEL_ERROR_RANGE = 8,
  DYNAMIXEL_ERROR_CHECKSUM = 16,
  DYNAMIXEL_ERROR_OVERLOAD = 32,
  DYNAMIXEL_ERROR_UNKNOWN = 64
} RoveDynamixelError;

void RoveDynamixel_begin(RoveDynamixel* dyna, RoveDynamixelType type, uint8_t id, uint8_t uart_index, int baud);

void RoveDynamixel_sendPacket(RoveDynamixel dyna, uint8_t length, uint8_t* instruction);
uint8_t RoveDynamixel_getReturnPacket(RoveDynamixel dyna, uint8_t* buffer, size_t buffer_size);
uint8_t RoveDynamixel_getError(RoveDynamixel dyna);

uint8_t RoveDynamixel_ping(RoveDynamixel dyna);
void RoveDynamixel_sendWriteCommand(RoveDynamixel dyna, uint8_t dynamixel_register, uint8_t data_length, uint8_t* data);
void RoveDynamixel_sendReadCommand(RoveDynamixel dyna, uint8_t dynamixel_register, uint8_t read_length);

uint8_t RoveDynamixel_rotateJoint(RoveDynamixel dyna, uint16_t speed, uint16_t position);
uint8_t RoveDynamixel_spinWheel(RoveDynamixel dyna, uint16_t speed);

uint8_t RoveDynamixel_setPosition(RoveDynamixel dyna, uint16_t position);
uint8_t RoveDynamixel_setSpeed(RoveDynamixel dyna, uint16_t speed);

uint8_t RoveDynamixel_setId(RoveDynamixelHandle dyna, uint8_t id);
uint8_t RoveDynamixel_setBaudRate(RoveDynamixel dyna, uint8_t baud_byte);
uint8_t RoveDynamixel_setReturnDelayTime(RoveDynamixel dyna, uint8_t return_delay_byte);
uint8_t RoveDynamixel_setMaxTorque(RoveDynamixel dyna, uint16_t max_torque);
uint8_t RoveDynamixel_setStatusReturnLevel(RoveDynamixel dyna, uint8_t level);
uint8_t RoveDynamixel_setMode(RoveDynamixel dyna, RoveDynamixelMode mode);

uint8_t RoveDynamixel_getMode(RoveDynamixel dyna, RoveDynamixelMode* mode);
uint8_t RoveDynamixel_getPresentPosition(RoveDynamixel dyna, uint16_t* pos);
uint8_t RoveDynamixel_getPresentSpeed(RoveDynamixel dyna, uint16_t* speed);
uint8_t RoveDynamixel_getLoad(RoveDynamixel dyna, uint16_t* load);
uint8_t RoveDynamixel_getVoltage(RoveDynamixel dyna, uint8_t* voltage);
uint8_t RoveDynamixel_getTemperature(RoveDynamixel dyna, uint8_t* temp);

#endif
