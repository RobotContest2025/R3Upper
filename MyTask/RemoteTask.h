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

#define Remote_BT_0_WIFI_1 1

#if !Remote_BT_0_WIFI_1

#pragma pack(1)

typedef struct
{
  uint16_t Left_Key_Up : 1;
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
  uint16_t Left_Switch_Up : 1;
  uint16_t Left_Switch_Down : 1;
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
  uint16_t Right_Switch_Up : 1;
  uint16_t Right_Switch_Down : 1;
} hw_key_t;

typedef struct
{
  uint8_t head;
  int16_t rocker[4];
  hw_key_t Key;
  uint8_t end;
} UART_DataPack;
#pragma pack()
#else
#pragma pack(1)
typedef struct
{
  uint8_t Left_Key_Up;
  uint8_t Left_Key_Down;
  uint8_t Left_Key_Left;
  uint8_t Left_Key_Right;
  uint8_t Left_Switch_L;
  uint8_t Left_Switch_R;
  uint8_t UNUSED1;
  uint8_t UNUSED2;

  uint8_t Right_Key_Up;
  uint8_t Right_Key_Down;
  uint8_t Right_Key_Left;
  uint8_t Right_Key_Right;
  uint8_t Right_Switch_R;
  uint8_t Right_Switch_L;
  uint8_t UNUSED3;
  uint8_t UNUSED4;
} hw_key_t;

typedef struct
{
  uint8_t head;
  uint16_t rocker[4];
  hw_key_t Key;
  uint8_t end;
} UART_DataPack;
#pragma pack()
#endif

/*typedef struct
{
  uint8_t head;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  uint8_t check;
} JY61P_t;*/

#pragma pack()

typedef enum
{
  ROBOT_RELATIVE, // 自由运动
  ROBOT_ABSOLUTE,
  ROBOT_STOP,        // 急停
  ROBOT_READY_LUNCH, // 以篮筐为圆心，面朝篮筐，锁定篮筐运动
} Move_Mode_t;

void RemoteTask(void *param);

#endif
