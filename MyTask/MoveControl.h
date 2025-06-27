#ifndef __MOVECONTROL_H__
#define __MOVECONTROL_H__

#include <stdio.h>
#include <stdint.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "Vague_PID.h"
#include "kalman.h"


//#define PI 3.14159265f
#define RAD2ANGLE(x) ((x)*180/PI)
#define ANGLE2RAD(x) ((x)*PI/180)

typedef struct
{
    float x;
    float y;
} Point_t;

#pragma pack(1) 

typedef struct
{
    uint8_t head;
    float v_x;
    float v_y;
    float omega;
    float acc;

    uint32_t cmd;    //附加的命令/参数
} ChassisCtrl_t;

typedef struct{
    float Attack_X_BallHoop,     
          Attack_Y_BallHoop,     
          Defend_X_BallHoop,     
          Defend_Y_BallHoop;
}Side_Data_Typedef;

typedef struct {
    float X, Y, Z;
} velocity_Ex_Typedef;	


typedef struct
{
    float x;            //绝对坐标系下机器人的位置
    float y;
    float angle;

    float v_x;          //机器人的速度
    float v_y;

    float omega;        //自旋角速度
    float Distance;     //距离目标篮筐的距离 
} RobotState_t; //机器人运动状态结构体(全部使用国际单位)

typedef struct
{
  uint8_t head;
  float v_x;
  float v_y;
  float omega;

  uint32_t state;
} ChassisState_t;   //底盘状态反馈

#pragma pack()

float reform(float X);
void MoveControlTask(void* param);

#endif
