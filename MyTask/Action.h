#ifndef __ACTION_H__
#define __ACTION_H__

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "motorEx.h"
#include "slope.h"

#define PI 3.14159265f
#define RAD2ANGLE(x) ((x)*180/PI)
#define ANGLE2RAD(x) ((x)*PI/180)

#define ABS(x) ((x > 0) ? x : -x)

#define TARGET_TRACK_UPDATE_TIME    2

#define ACTION_TYPE_INTERRUPTABLE 0x00
#define ACTION_TYPE_UNINTERRUPTABLE 0x01


#define ActionFinished()	\
		*((uint8_t*)param)=1;\
		xSemaphoreGive(action_semaphore);\
		vTaskDelete(NULL)


typedef void(*ActionCallback_t)(void* param);

typedef struct
{
    uint32_t type;
    ActionCallback_t action_cb;
    void* param;
}Action_t;

typedef struct
{
    float distance;
    float jump_motor_iq;
    float throw_motor_torque;
    float detach_angle;
    float break_angle;
}LaunchDots_t;


#define CMD_MODE_MOVE               1     //主桅杆上升，舵机操纵主桅杆锁定，允许自由移动
#define CMD_MODE_JUMP               2     //舵机操纵主桅杆取消锁定，主桅杆下降，轮子离地，准备跳跃
#define CMD_MODE_LOCK_CHASSIS       3     //锁定上下小底盘
#define CMD_MODE_UNLOCK_CHASSIS     4     //解除锁定上下小底盘
#define CMD_MODE_LAUNCH             5     ///舵机操纵主桅杆取消锁定，主桅杆和轮子同时触地，同时轮子内八锁定防止车辆发射时旋转


//动作
void DetachClawAction(void* param);
void MeasureTask(void* param);


void ResetAction(void* param);     //机器人复位到准备上场的状态，通常上电后等待各部件全部复位完成后自动执行，也可由操作手手动执行
void DownChassis(void* param);			//放下底盘


void ReadyBackThrowAction(void *param);
void BackThrowAction(void *param);


void ReadyDribbleAction2(void *param);
void DribbleAction2(void* param);

void ReadyJumpAction(void* param);
void JumpAction(void* param);

void DefendAction(void* param);
	
void PickUpBallAction(void* param);



//辅助函数
float GravityTorqueCompensation(float rad,float omega,float aerfa,uint8_t has_ball);
uint16_t SetSteeringEngineRAD180(float rad);
uint16_t SetSteeringEngineRAD270(float rad);
void MotorTargetTrack(int target, int *ctrl_var, int rate);
void MotorTargetTrack_float(float target, float *ctrl_var, float rate);
LaunchDots_t CalculateLaunchParam(float distance);
float TargetSlope(float target,float current,float max_increase,float min_reduction);


#endif