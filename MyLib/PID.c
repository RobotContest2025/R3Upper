#include "PID.h"


/***************************PID param initialize******************************/
static void f_PID_param_init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float Changing_Integral_A,
    float Changing_Integral_B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Output_Filtering_Coefficient = output_filtering_coefficient;

    pid->Derivative_Filtering_Coefficient = derivative_filtering_coefficient;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}
/**************************PID param reset*********************************/
static void f_PID_reset(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    if (pid->Ki == 0)
		pid->Iout = 0;
}
/*****************PID structure initialize*********************/
void PID_Init(
    PID_TypeDef *pid,

    float kp,
    float Ki,
    float Kd,
		
    uint16_t intergral_limit,
    float deadband,
    uint16_t max_out,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset 		= f_PID_reset;
    pid->PID_param_init(pid, max_out, intergral_limit, deadband,
                        kp, Ki, Kd, A, B, output_filtering_coefficient, derivative_filtering_coefficient, improve);
}

/***************************PID calculate**********************************/
float PID_Calculate(float measure, float target, PID_TypeDef *pid)
{
/*    if (pid->Improve & ErrorHandle) //ErrorHandle
    {
        f_PID_ErrorHandle(pid); // last_xxx = xxx
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0; //Catch ERROR
        }
    }
*/
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);
       
        if (pid->Improve & Derivative_On_Measurement) //Derivative On Measurement
			pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
		
        if (pid->Improve & DerivativeFilter) //Derivative filter
			pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient+ pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
    }
	pid->ITerm = pid->Ki * pid->Err;

	if (pid->Improve & Trapezoid_Intergral) //Trapezoid Intergral
	pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);

	if (pid->Improve & ChangingIntegralRate){ //Changing Integral Rate
		if ((pid->Err * pid->Iout > 0) && (ABS(pid->Err) > pid->ScalarB))
		{   //Integral still increasing
			if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
				pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
			else
				pid->ITerm = 0;
		}
	}

	if (pid->Improve & Integral_Limit){ //Integral limit
		float temp_Output, temp_Iout;
		temp_Iout = pid->Iout + pid->ITerm;
		temp_Output = pid->Pout + pid->Iout + pid->Dout;
		if ((ABS(temp_Output) > pid->MaxOut) && (pid->Err * pid->Iout > 0))//Integral still increasing
			pid->ITerm = 0;
		
		if (temp_Iout > pid->IntegralLimit){
			pid->ITerm = 0;
			pid->Iout = pid->IntegralLimit;
		}
		if (temp_Iout < -pid->IntegralLimit){
			pid->ITerm = 0;
			pid->Iout = -pid->IntegralLimit;
		}
	}
	
	pid->Iout += pid->ITerm;
		
	pid->Output = pid->Pout + pid->Iout + pid->Dout;

	if (pid->Improve & OutputFilter) //Output Filter
		pid->Output = pid->Output * pid->Output_Filtering_Coefficient + pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);

	if (pid->Output > pid->MaxOut)
		pid->Output = pid->MaxOut;
	if (pid->Output < -(pid->MaxOut))
		pid->Output = -(pid->MaxOut);

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;

    return pid->Output;
}
float PID_Control_Increment(float measure, float target,PID_TypeDef *pid) {
	pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err - pid->Last_Err;
        pid->ITerm = pid->Ki * pid->Err;
        pid->Dout = pid->Kd * (pid->Err - 2 * pid->Last_Err + pid->Before_Last_Err);

       
        if (pid->Improve & Integral_Limit){ //Integral limit
			float temp_Output = pid->Pout + pid->Iout + pid->Dout, temp_Iout = pid->ITerm;
			if (ABS(temp_Output) > pid->MaxOut)
				if (pid->Err * pid->Iout > 0)	pid->ITerm = 0;//Integral still increasing
			if (temp_Iout > pid->IntegralLimit){
				pid->ITerm = 0;
				pid->Iout = pid->IntegralLimit;
			}
			if (temp_Iout < -pid->IntegralLimit){
				pid->ITerm = 0;
				pid->Iout = -pid->IntegralLimit;
			}
		}

        if (pid->Improve & DerivativeFilter) //Derivative filter
			pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient+ pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);

        pid->Iout = pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        
        if (pid->Improve & OutputFilter) //Output Filter
			pid->Output = pid->Output * pid->Output_Filtering_Coefficient + pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);

		if (pid->Output > pid->MaxOut)
			pid->Output = pid->MaxOut;
		if (pid->Output < -(pid->MaxOut))
			pid->Output = -(pid->MaxOut);
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Before_Last_Err = pid->Last_Err;

    return pid->Output;	
}
float PID_Control_Smis(float measure, float target,PID_TypeDef *pid, float speed) {
	pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    if (ABS(pid->Err) > pid->DeadBand)
    {
        pid->Pout = pid->Kp * pid->Err;
        pid->Dout = pid->Kd * speed;
		
        if (pid->Improve & DerivativeFilter) //Derivative filter
			pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient+ pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);

    }
	pid->ITerm = pid->Ki * pid->Err;

	if (pid->Improve & Trapezoid_Intergral) //Trapezoid Intergral
		pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
       
	if (pid->Improve & ChangingIntegralRate){ //Changing Integral Rate
		if ((pid->Err * pid->Iout > 0) && (ABS(pid->Err) > pid->ScalarB))
		{	//Integral still increasing
			if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
				pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
			else
				pid->ITerm = 0;
		}
	}
	if (pid->Improve & Integral_Limit){ //Integral limit
		float temp_Output, temp_Iout;
		temp_Iout = pid->Iout + pid->ITerm;
		temp_Output = pid->Pout + pid->Iout + pid->Dout;
		if ((ABS(temp_Output) > pid->MaxOut) && (pid->Err * pid->Iout > 0))//Integral still increasing
			pid->ITerm = 0;

		if (temp_Iout > pid->IntegralLimit)
		{
			pid->ITerm = 0;
			pid->Iout = pid->IntegralLimit;
		}
		if (temp_Iout < -pid->IntegralLimit)
		{
			pid->ITerm = 0;
			pid->Iout = -pid->IntegralLimit;
		}
	}

	pid->Iout += pid->ITerm;

	
	pid->Output = pid->Pout + pid->Iout + pid->Dout;
   
	
	if (pid->Improve & OutputFilter) //Output Filter
		pid->Output = pid->Output * pid->Output_Filtering_Coefficient + pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);

	if (pid->Output > pid->MaxOut)
		pid->Output = pid->MaxOut;
	if (pid->Output < -(pid->MaxOut))
		pid->Output = -(pid->MaxOut);

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;

    return pid->Output;
}

/*****************PID ERRORHandle Function*******************/  /*
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    //Motor Blocked Handle
    if (pid->Output < pid->MaxOut * 0.01f)
        return;

    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        //Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}*/

