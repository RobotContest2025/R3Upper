#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/**
 * @brief 限幅宏函数
 * @param IN 限幅变量
 * @param MAX 最大值
 * @param MIN 最小值
 */
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;            \
    if (IN > MAX)           \
        IN = MAX

#include "stdint.h"
#include "main.h"

#define ABS(x) ((x > 0) ? x : -x)

				 
/******************************* PID CONTROL *********************************/					 
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000	 无使能
    Integral_Limit = 0x01,              //0000 0001  积分限幅
    Derivative_On_Measurement = 0x02,   //0000 0010  微分先行
    Trapezoid_Intergral = 0x04,         //0000 0100  梯形积分
    Proportional_On_Measurement = 0x08, //0000 1000  比例测量(禁用)
    OutputFilter = 0x10,                //0001 0000  输出滤波
    ChangingIntegralRate = 0x20,        //0010 0000  变速积分
    DerivativeFilter = 0x40,            //0100 0000  衍生滤波
    ErrorHandle = 0x80,                 //1000 0000  异常处理
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef
{
    float Kp;
    float Ki;
    float Kd;

    float DeadBand;
    float IntegralLimit;    //积分限幅
    float Err;
    float Last_Err;
    float Output;
	
    float Pout;
    float ITerm;
    float Iout;
    float Dout;
    float MaxOut;
	
    float Measure;      // 测量值
    float Last_Measure;
	float Before_Last_Err;
    float Target;

    float Last_Output;
    float Last_Dout;

//    float ControlPeriod;
    float ScalarA; //For Changing Integral
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_Filtering_Coefficient;//越大，受上一次输出值影响的越大
    float Derivative_Filtering_Coefficient;//0-1的分数,越小受上一次的微分结果影响越大

//    uint32_t thistime;
//    uint32_t lasttime;
//    uint8_t dtime;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        uint16_t maxOut,
        uint16_t integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef;


static void f_PID_ErrorHandle(PID_TypeDef *pid);

void PID_Init(
    PID_TypeDef *pid,

    float kp,
    float ki,
    float kd,
		
    uint16_t intergral_limit,
    float deadband,
    uint16_t max_out,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve);

float PID_Calculate(float measure, float target,PID_TypeDef *pid);
float PID_Control_Smis(float measure, float target,PID_TypeDef *pid, float speed);
float PID_Control_Increment(float measure, float target,PID_TypeDef *pid);		

	

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

	
typedef struct  // 模糊PID结构体
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

	float KpMax;
    float KiMax;
    float KdMax;

    float KpRatio;
    float KiRatio;
    float KdRatio;
	float integral;
	float IntegralLimit;
	
    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;
		
	float dt;

    float max_output;

}FuzzyRule_t;

void Fuzzy_Rule_Init(float kpMax, float kiMax, float kdMax, float integralLimit, float eStep,float ecStep, float dt,float max_output,FuzzyRule_t *fuzzyRule);
float Fuzzy_Rule_Implementation(float measure, float target, FuzzyRule_t *fuzzyRule);

		
	
#endif
#ifdef __cplusplus
}
#endif

