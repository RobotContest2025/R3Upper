#ifndef __SLOPE_H__
#define __SLOPE_H__


#define MOTOR_SLOPE_MODE_LINE   1
#define MOTOR_SLOPE_MODE_ELINE  2
#define MOTOR_SLOPE_MODE_SLINE  3

#include "stm32f4xx_hal.h"


//@brief 电机斜坡控制参数
typedef struct 
{
  float max_change_rate;  //最大速度变化率
  float switch_point;     //转折点
  float speed_change_rate;//当前的速度改变率
  float alphe;            //速度变化率迭代参数（模式3，S型使用）
  float died_space;       //模式3（S型曲线）死区控制
  uint8_t type;           //电机斜坡控制器类型（模式1—线性，模式2—指数函数型，模式3—S曲线型）
}MoterSlope_t;



void MoterSlope(float target,float* now_target,MoterSlope_t* slopeSet);
void Vector2dSlope(float *target,float* now_target,float max_rate);

#endif

//参数配置参考
/*
  MoterSlopeStructure.max_change_rate=10.0f;    //直线型
  MoterSlopeStructure.type=MOTOR_SLOPE_MODE_LINE;
*/
/*
  MoterSlopeStructure.max_change_rate=30.0f;    //S型曲线型
  MoterSlopeStructure.type=MOTOR_SLOPE_MODE_SLINE;
  MoterSlopeStructure.alphe=2.0f;
  MoterSlopeStructure.died_space=30.0f;
  //MoterSlopeStructure.switch_point=1;
  //MoterSlopeStructure.fresh_ticks=1;
*/
/*
  MoterSlopeStructure.max_change_rate=30.0f;    //指数曲线型
  MoterSlopeStructure.type=MOTOR_SLOPE_MODE_ELINE;
  MoterSlopeStructure.alphe=0.01f;
  MoterSlopeStructure.died_space=20.0f;
  MoterSlopeStructure.switch_point=600.0f;
  //MoterSlopeStructure.fresh_ticks=1;
*/
