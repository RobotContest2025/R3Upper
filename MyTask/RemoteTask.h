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

typedef enum 
{
    ROBOT_IDEL,   //�ϵ���״̬,10s��ִ�и�λ����MOVE״̬
    ROBOT_MOVE,
    ROBOT_DEFEND,
    ROBOT_PICKUP,

    ROBOT_READY_LAUNCH,
    ROBOT_LAUNCH,
    ROBOT_READY_JUMP,
    ROBOT_JUMP,
    ROBOT_READY_DRIBBLE,
    ROBOT_DRIBBLE
}RobotMode;

typedef struct{
    RobotMode Mode;
    uint8_t Action_Finish;
    uint8_t Robot_Init;
} RobotMode_Typedef;


#pragma pack(1)

typedef struct{
   uint8_t Left_Key_Up : 1;         
   uint8_t Left_Key_Down : 1;       
   uint8_t Left_Key_Left : 1;       
   uint8_t Left_Key_Right : 1;       
   uint8_t Left_Switch_Up_or_Left : 1;       
   uint8_t Left_Switch_Down_or_Right: 1;       
   uint8_t UNUSED1 : 1;
   uint8_t UNUSED2 : 1;

   uint8_t Right_Key_Up : 1;        
   uint8_t Right_Key_Down : 1;      
   uint8_t Right_Key_Left : 1;      
   uint8_t Right_Key_Right : 1;     
   uint8_t Right_Switch_Up_or_Right : 1;      
   uint8_t Right_Switch_Down_or_Left : 1;      
   uint8_t UNUSED3 : 1; 
   uint8_t UNUSED4 : 1;
} hw_key_t;


typedef struct{
	uint8_t head;
	int16_t rocker[4];
	hw_key_t Key;
	uint8_t end;
} UART_DataPack;


typedef struct
{
	uint8_t head;
	uint8_t type;
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t t;
	uint8_t check;
}JY61P_AccSensor_t;


typedef struct {
    int8_t head;    //0X2B
    int16_t angle;  //��*180    
    int16_t x;     	//mm
    int16_t y;	    //mm
}PositionPack_Typedef;		//�״ﶨλ���ݰ�

typedef struct {
    float angle,    //��
          x,     	//mm
          y;	    //mm
}Position_Typedef;		//�״ﶨλ���ݰ�
#pragma pack()


void RemoteTask(void *param);

#endif
