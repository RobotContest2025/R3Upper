#include "Vague_PID.h"


static float FuzzyRuleKpRAW[7][7] = {
    PB, PB, PM, PM, PB, PB, PM,
    PB, PM, PS, PS, PM, PM, PS,
    PM, PS, PS, ZE, PS, PS, ZE,
    PS, ZE, ZE, NM, ZE, ZE, PS,
    ZE, PS, PS, ZE, PS, PS, PM,
    PS, PM, PM, PS, PS, PM, PB,
    PM, PB, PB, PM, PM, PB, PB};

static float FuzzyRuleKiRAW[7][7] = {
    NB, NM,	NS, NM, NM, NM, ZE,
    NM, ZE,	PS, ZE, NS, ZE, PS,
    ZE, PM,	PM, PM, PS, PM, PB,
    PB, PB,	PB, PB, PB, PB, PB,
    PB, PM,	PS, PM, PM, PM, ZE,
    PM, ZE,	NS, ZE, PS, ZE, NM,
    ZE, NM,	NM, NM, NS, NM, NB};

static float FuzzyRuleKdRAW[7][7] = {
    PB, PM, PS, NB, NB, NM, PS,
    PM, PS, ZE, NM, NM, NS, ZE,
    PS, ZE, ZE, NM, NS, NS, ZE,
    PS, ZE, NS, NS, NS, ZE, PS,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PS, ZE, PS, PS, PS, PS, PM,
    PS, PS, PM, PM, PM, PM, PB};



uint8_t Fuzzy_Rule_Init(float kpMax, float kiMax, float kdMax, float integralLimit, float eStep, float max_out, float deadband, FuzzyRule_t *fuzzyRule)
{
    fuzzyRule->KpMax = kpMax;
    fuzzyRule->KiMax = kiMax;
    fuzzyRule->KdMax = kdMax;
	fuzzyRule->IntegralLimit = integralLimit;
	fuzzyRule->max_out = max_out;
	fuzzyRule->deadband = deadband;

    fuzzyRule->KpRatio = 0;
    fuzzyRule->KiRatio = 0;
    fuzzyRule->KdRatio = 0;
    fuzzyRule->integral = 0;
    
    if (eStep < 0.00001f)
        eStep = 1;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = eStep;

    return 1;
}
float Fuzzy_Rule_Implementation(float measure, float target, FuzzyRule_t *fuzzyRule)
{
    float eLeftTemp,  ecLeftTemp,
          eRightTemp, ecRightTemp;
    uint8_t eLeftIndex,  ecLeftIndex,
            eRightIndex, ecRightIndex;

    fuzzyRule->e = target - measure;
	float dt = DWT_GetDeltaT((void *)&fuzzyRule->Fuzzy_DWT_CNT);
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast)/dt;
    fuzzyRule->eLast = fuzzyRule->e;

    if (fabsf(fuzzyRule->e) >= fuzzyRule->deadband){
        //Á¥ÊôÇø¼ä
        eLeftIndex   = fuzzyRule->e  >= fuzzyRule->eStep  ? 6 : (fuzzyRule->e  <= -fuzzyRule->eStep  ? 0 : (fuzzyRule->e  >= 0 ? ((int)(fuzzyRule->e  / (fuzzyRule->eStep  / 3.f)) + 3) : ((int)(fuzzyRule->e  / (fuzzyRule->eStep  / 3.f)) + 2)));
        eRightIndex  = fuzzyRule->e  >= fuzzyRule->eStep  ? 6 : (fuzzyRule->e  <= -fuzzyRule->eStep  ? 0 : (fuzzyRule->e  >= 0 ? ((int)(fuzzyRule->e  / (fuzzyRule->eStep  / 3.f)) + 4) : ((int)(fuzzyRule->e  / (fuzzyRule->eStep  / 3.f)) + 3)));
        ecLeftIndex  = fuzzyRule->ec >= fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / (fuzzyRule->ecStep / 3.f)) + 3) : ((int)(fuzzyRule->ec / (fuzzyRule->ecStep / 3.f)) + 2)));
        ecRightIndex = fuzzyRule->ec >= fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / (fuzzyRule->ecStep / 3.f)) + 4) : ((int)(fuzzyRule->ec / (fuzzyRule->ecStep / 3.f)) + 3)));

        //Á¥Êô¶È
        eLeftTemp   = eRightIndex  == 0 ? 1 : (eLeftIndex   == 6 ? 0 : (eRightIndex - fuzzyRule->e / (fuzzyRule->eStep / 3.f) - 3));
        eRightTemp  = eLeftIndex   == 6 ? 1 : (eRightIndex  == 0 ? 0 : (fuzzyRule->e / (fuzzyRule->eStep / 3.f) - eLeftIndex + 3));
        ecLeftTemp  = ecRightIndex == 0 ? 1 : (ecLeftIndex  == 6 ? 0 : (ecRightIndex - fuzzyRule->ec / (fuzzyRule->ecStep / 3.f) - 3));
        ecRightTemp = ecLeftIndex  == 6 ? 1 : (ecRightIndex == 0 ? 0 : (fuzzyRule->ec / (fuzzyRule->ecStep / 3.f) - ecLeftIndex + 3));

        fuzzyRule->KpFuzzy = eLeftTemp  * ecLeftTemp  * FuzzyRuleKpRAW[eLeftIndex ][ecLeftIndex ]+
                             eLeftTemp  * ecRightTemp * FuzzyRuleKpRAW[eRightIndex][ecLeftIndex ]+
                             eRightTemp * ecLeftTemp  * FuzzyRuleKpRAW[eLeftIndex ][ecRightIndex]+
                             eRightTemp * ecRightTemp * FuzzyRuleKpRAW[eRightIndex][ecRightIndex];

        fuzzyRule->KiFuzzy = eLeftTemp  * ecLeftTemp  * FuzzyRuleKiRAW[eLeftIndex ][ecLeftIndex ]+
                             eLeftTemp  * ecRightTemp * FuzzyRuleKiRAW[eRightIndex][ecLeftIndex ]+
                             eRightTemp * ecLeftTemp  * FuzzyRuleKiRAW[eLeftIndex ][ecRightIndex]+
                             eRightTemp * ecRightTemp * FuzzyRuleKiRAW[eRightIndex][ecRightIndex];

        fuzzyRule->KdFuzzy = eLeftTemp  * ecLeftTemp  * FuzzyRuleKdRAW[eLeftIndex ][ecLeftIndex ]+
                             eLeftTemp  * ecRightTemp * FuzzyRuleKdRAW[eRightIndex][ecLeftIndex ]+
                             eRightTemp * ecLeftTemp  * FuzzyRuleKdRAW[eLeftIndex ][ecRightIndex]+
                             eRightTemp * ecRightTemp * FuzzyRuleKdRAW[eRightIndex][ecRightIndex];
                             
        fuzzyRule->KpRatio = fuzzyRule->KpMax * (fuzzyRule->KpFuzzy + 3) / 6.f;
        fuzzyRule->KiRatio = fuzzyRule->KiMax * (fuzzyRule->KiFuzzy + 3) / 6.f;
        fuzzyRule->KdRatio = fuzzyRule->KdMax * (fuzzyRule->KdFuzzy + 3) / 6.f;

        fuzzyRule->integral += fuzzyRule->KiRatio * fuzzyRule->e * dt;
        limit(fuzzyRule->integral, fuzzyRule->IntegralLimit, -fuzzyRule->IntegralLimit);
        
        fuzzyRule->Output = fuzzyRule->KpRatio * fuzzyRule->e + fuzzyRule->integral + fuzzyRule->KdRatio * fuzzyRule->ec;
        limit(fuzzyRule->Output, fuzzyRule->max_out, -fuzzyRule->max_out);
	}
    else {
        fuzzyRule->integral = 0;
        fuzzyRule->Output = 0;
        return 0;
    }
	return fuzzyRule->Output;		
}

//float Inverse_quantization(float maximum, float minimum, float qvalues)
//{
//	float x = (maximum - minimum) * (qvalues + 3) / 6 + minimum;
//	return x;
//}

