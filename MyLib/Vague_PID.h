#ifndef _Vague_PID_H_
#define _Vague_PID_H_


#include "stdint.h"
#include "math.h"
#include "bsp_dwt.h"
#include "PID.h"

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

	float Output;
    
    float max_out;
    float deadband;
    
    float eLast;

    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

	uint32_t Fuzzy_DWT_CNT;
}FuzzyRule_t;

uint8_t Fuzzy_Rule_Init(float kpMax, float kiMax, float kdMax, float integralLimit, float eStep, float max_out, float deadband, FuzzyRule_t *fuzzyRule);
float Fuzzy_Rule_Implementation(float measure, float target, FuzzyRule_t *fuzzyRule);

	
#endif
#ifdef __cplusplus
}

#endif
