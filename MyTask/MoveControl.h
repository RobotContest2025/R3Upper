#ifndef __MOVECONTROL_H__
#define __MOVECONTROL_H__

#include <stdio.h>
#include <stdint.h>
//#include "arm_math.h"
#include "matrix.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

#define PI 3.14159265f
#define RAD2ANGLE(x) (x*180/PI)
#define ANGLE2RAD(x) (x*PI/180)

typedef struct
{
    float x;
    float y;
} Point_t;

typedef struct
{
    float v_x;
    float v_y;
    float omega;
} ChassisCtrl_t; //底盘控制结构体(全部使用国际单位制)

#pragma pack(1) 
typedef struct
{
    float x;        //绝对坐标系下机器人的位置
    float y;
    float angle;

    float v_x;      //绝对坐标系下机器人的速度
    float v_y;

	float iner_vx;  //机器人坐标系下的速度
    float iner_vy;

    float omega;  //自旋角速度
} RobotState_t; //机器人运动状态结构体(全部使用国际单位)


typedef struct {
    int16_t X;
    int16_t Y;
    float Angle,
          X_Velocity,//m/s
          Y_Velocity,
          Z_Velocity,	//度/s
          Body_X_Velocity,	//m/s
          Body_Y_Velocity;
} PositionPack_Typedef;

#pragma pack()

void MoveControlTask(void* param);

#endif
