#ifndef __REMOTETASK_H__
#define __REMOTETASK_H__

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "WatchDog2.h"
#include "Action.h"
#include "motorControl.h"
#include "kalman.h"
#include "usart.h"

#include <stdio.h>

typedef struct
{
  uint16_t Left_Key_Up : 1;          //将遥控器的状态赋值到这个结构体中
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;        //位域
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
//  uint16_t Left_Switch : 2;
	uint16_t Left_Switch_up : 1;
	uint16_t Left_Switch_down : 1;
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
//  uint16_t Right_Switch : 2;
	uint16_t Right_Switch_up : 1;
	uint16_t Right_Switch_down : 1;
} hw_key_t;

#pragma pack(1)       //地址对齐，为了让数据包中的数据不会乱，
typedef struct {
  uint8_t head;           
  uint16_t rocker[4];
  hw_key_t Key;
  int32_t Left_Encoder;
  int32_t Right_Encoder;
  uint16_t crc;
} UART_DataPack;
#pragma pack()

typedef enum
{
  ROBOT_RELATIVE,   //自由运动
  ROBOT_ABSOLUTE,
  ROBOT_STOP,   //急停
  ROBOT_READY_LUNCH,   //以篮筐为圆心，面朝篮筐，锁定篮筐运动
}Move_Mode_t;

void RemoteTask(void *param);

#endif
