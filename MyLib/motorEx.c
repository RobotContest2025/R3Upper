#include "motorEx.h"


uint32_t Motor3508Recv(Motor3508Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf)
{
    if(hcan->Instance==motor_ex->hcan->Instance&&ID==motor_ex->ID)
    {
        RM3508_Receive(&(motor_ex->motor),buf);
        if(!motor_ex->ready)
        {
            motor_ex->ready=1;
            motor_ex->offset=motor_ex->motor.Angle;
        }
        motor_ex->actual_pos=motor_ex->motor.Angle-motor_ex->offset;
        return 1;
    }
    return 0;
}

uint32_t Motor2006Recv(Motor2006Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf)
{
    if(hcan->Instance==motor_ex->hcan->Instance&&ID==motor_ex->ID)
    {
        M2006_Receive(&(motor_ex->motor),buf);
        if(!motor_ex->ready)
        {
            motor_ex->ready=1;
            motor_ex->offset=motor_ex->motor.Angle;
        }
        motor_ex->actual_pos=motor_ex->motor.Angle-motor_ex->offset;
        return 1;
    }
    return 0;
}
