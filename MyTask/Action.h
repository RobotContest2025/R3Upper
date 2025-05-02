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
#define RAD2ANGLE(x) (x*180/PI)
#define ANGLE2RAD(x) (x*PI/180)

#define ABS(x) ((x > 0) ? x : -x)

#define TARGET_TRACK_UPDATE_TIME    2


typedef struct
{
    int32_t distance;  //mm
    int32_t last_distance;
    uint8_t state;
}LV53_Sensor_t;

typedef void(*ActionCallback_t)(void* param);

typedef struct
{
    void* param;
    ActionCallback_t action_cb;
}Action_t;


uint16_t SetSteeringEngineRAD180(float rad);
uint16_t SetSteeringEngineRAD270(float rad);
uint32_t MotorTargetTrack(int target, int *ctrl_var, int rate, uint32_t *exit_ctrl);
float MotorTargetTrack_float(float target, float *ctrl_var, float rate, uint32_t *exit_ctrl);


//动作
void ResetAction(void* param);
void ClutchLockAction(void* param);
void TestAction2(void *param);
void TestAction(void* param);
void ReadyDribbleAction(void* param);
void ReadyLaunchAction(void* param);
void DribbleAction(void *param);
void LaunchAction(void *param);
void MainBodyDeclineAction(void* param);


#endif