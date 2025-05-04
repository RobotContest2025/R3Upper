#ifndef __ROBSTRIDE2_H__
#define __ROBSTRIDE2_H__

#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define HOST_ID 0xFD

#define PARAM_RUN_MODE           0x7005
#define PARAM_IQ_REF             0x7006
#define PARAM_SPD_REF            0x700A
#define PARAM_LIMIT_TORQUE       0x700B
#define PARAM_CUR_KP             0x7010
#define PARAM_CUR_KI             0x7011
#define PARAM_CUR_FILT_GAIN      0x7014
#define PARAM_LOC_REF            0x7016
#define PARAM_LIMIT_SPD          0x7017
#define PARAM_LIMIT_CUR          0x7018
#define PARAM_MECH_POS           0x7019
#define PARAM_IQF                0x701A
#define PARAM_MECH_VEL           0x701B
#define PARAM_VBUS               0x701C
#define PARAM_R_OTATION          0x701D
#define PARAM_LOC_KP             0x701E
#define PARAM_SPD_KP             0x701F
#define PARAM_SPD_KI             0x7020


#define M_PI 3.141592653589793

#define U16_FLIP(x) ((((uint16_t)(x))>>8)|(((uint16_t)(x))<<8))
#define DEPACK_U8_TO_U16(low,high) (((uint16_t)(low))|(uint16_t)((uint16_t)(high)<<8))
#define DEPACK_U8_TO_U16_FLIP(low,high) (((uint16_t)(high))|((uint16_t)((uint16_t)(low)<<8)))

#define MAX_REQUIRE_LIST_SIZE 4



typedef enum
{
    RobStride_01,
    RobStride_04
}RobStrideType;

typedef enum
{
    RobStride_MotionControl,
    RobStride_Position,
    RobStride_Speed,
    RobStride_Torque,
}RobStrideMode;

typedef struct
{
    uint32_t run_mode;
    float torque_limit;
    float cur_kp;
    float cur_ki;
    float cur_filt_gain;
    float limit_spd;
    float limit_cur;
    float loc_kp;
    float spd_kp;
    float spd_ki;
}RobStrideParam_t;


typedef struct
{
    float rad;
    float omega;
    float torque;
    float iqf;
    float r;
    float temperature;
    float vbus;
    uint64_t error;
    uint8_t feedback;
}RobStrideState_t;

typedef struct
{
    //使用的CAN口
    CAN_HandleTypeDef *hcan;
    uint32_t motor_id;              //电机8位ID
    uint32_t host_id;                //主机ID

    RobStrideType type;             //电机类型
    RobStrideState_t state;           //电机运行时信息
    RobStrideParam_t param;           //电机参数设置
}RobStride_t;

uint32_t RobStrideSend(RobStride_t *device,uint32_t ExtID,uint8_t* buf);

void RobStrideInit(RobStride_t *device, CAN_HandleTypeDef *hcan, uint32_t id, RobStrideMode mode, RobStrideType type);
uint32_t RobStrideSetMode(RobStride_t *device,RobStrideMode mode);
uint32_t RobStrideEnable(RobStride_t *device);
uint32_t RobStrideDisable(RobStride_t *device,uint8_t clear_error);
uint32_t RobStrideResetAngle(RobStride_t *device);
uint32_t RobStrideGet(RobStride_t *device,uint16_t cmd);


uint32_t RobStrideMotionControl(RobStride_t *device,float torque,float target_rad,float target_omega);
uint32_t RobStrideTorqueControl(RobStride_t *device,float req);
uint32_t RobStrideSpeedControl(RobStride_t *device,float vel);
uint32_t RobStridePositionControl(RobStride_t *device,float pos);
uint32_t RobStrideSetVelPID(RobStride_t *device,float kp,float ki);
uint32_t RobStrideSetLocPID(RobStride_t *device,float kp);
uint32_t RobStrideSetCurPID(RobStride_t *device,float kp,float ki);
uint32_t RobStrideSetVelLimit(RobStride_t *device,float vel);
uint32_t RobStrideSetCurLimit(RobStride_t *device,float cur);
uint32_t RobStrideSetTorqueLimit(RobStride_t *device,float torque);


//Base Function

uint32_t RobStrideRecv_Handle(RobStride_t* device,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf);

#endif