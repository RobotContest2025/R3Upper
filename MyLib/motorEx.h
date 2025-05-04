#ifndef __MOTOREX_H__
#define __MOTOREX_H__

#include "motor.h"
#include "pid_old.h"

typedef struct
{
    RM3508_TypeDef motor;
    CAN_HandleTypeDef *hcan;
    uint32_t ID;
    int32_t offset;
    int32_t actual_pos;
    PID2 pos_pid;
    PID2 vel_pid;
    uint8_t ready;
}Motor3508Ex_t;

typedef struct
{
    M2006_TypeDef motor;
    CAN_HandleTypeDef *hcan;
    uint32_t ID;
    int32_t offset;
    int32_t actual_pos;
    PID2 pos_pid;
    PID2 vel_pid;
    uint8_t ready;
}Motor2006Ex_t;

uint32_t Motor2006Recv(Motor2006Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf);
uint32_t Motor3508Recv(Motor3508Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf);



#endif
