#include "Action.h"
#include "Odrive.h"
#include "RobStride2.h"
#include "tim.h"


extern TIM_HandleTypeDef htim1;
extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;
extern QueueHandle_t action_semaphore;

// 传感器应用
uint8_t lv53_recv_buf[64];
LV53_Sensor_t lv53_sensor;

uint8_t lv53_recv_buf_chass[64];
LV53_Sensor_t lv53_sensor_chass;

extern float chassis_v;

extern uint32_t task_exit;

extern Motor2006Ex_t claw_motor;
extern Motor3508Ex_t push_motor;
extern ODrive jump_motor1;
extern ODrive jump_motor2;
extern RobStride_t throw_motor;
extern float throw_motor_actual_pos;

extern uint8_t throw_motor_vel_mode;
extern uint8_t jump_motor_vel_mode;
extern uint8_t jump_motor_enable;
extern int32_t claw_motor_target_pos;
extern float throw_motor_target_vel;
extern float throw_motor_target_iq;
extern float jump_motor_target_vel;
extern float jump_motor_target_cur;
extern float lift_motor1_target_vel;
extern float lift_motor2_target_vel;


float lv53_gate_dis = 270.0f;
float debug_value = 0.0f;
int32_t launch_velocity = -60.0f;
int32_t start_launch_time = 1100;


void ResetAction(void* param)	//上电时执行一次的动作，将机械结构件复位到正确的位置
{
	UNUSED(param);
	jump_motor_enable=1;
	ActionFinished();
}


float unlock_rad1=4.10f;
float lock_rad1=3.54f;
float unlock_rad2=3.56f;
float lock_rad2=3.0f;
void TestAction(void *param)		//解锁锁定
{
	UNUSED(param);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SetSteeringEngineRAD270(unlock_rad1));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SetSteeringEngineRAD270(unlock_rad2));
	ActionFinished();
}

float throw_height=5.0f;
float throw_start_rad=0.0f;

float throw_start_torque=40.0f;
float throw_break_torque=-30.0f;
float throw_target_omega=3.0f;
float throw_detach_rad=2.8f;
float throw_break_rad=3.14f;

void ThrowAction(void* param)
{
	UNUSED(param);
	throw_motor_vel_mode=0;
	throw_motor_target_iq=throw_start_torque;
	while(throw_motor.state.omega<throw_target_omega)	//速度小于目标速度时一直以最大允许力矩驱动大臂加速
		vTaskDelay(3);
	
	throw_motor_target_vel=throw_target_omega;
	throw_motor_vel_mode=1;
	while(throw_motor.state.rad<throw_break_rad)	//重力补偿
	{
		throw_motor_target_iq=GravityCompensation(throw_motor.state.rad);
		vTaskDelay(3);
	}

	throw_motor_vel_mode=0;
	throw_motor_target_iq=throw_break_torque;
	while(throw_motor.state.omega>0.0f)			//等待刹车完成
		vTaskDelay(3);
	throw_motor_target_iq=0.0f;
	throw_motor_target_vel=0.0f;
	throw_motor_vel_mode=1;
	vTaskDelay(1000);

	throw_motor_target_vel=-1.0f;				//大臂复位
	while(throw_motor.state.rad>throw_start_rad)
		vTaskDelay(3);
	throw_motor_target_vel=0.0f;
	throw_motor_vel_mode=0;
	ActionFinished();
}

void ReadyThrowAction(void* param)
{
	throw_motor_vel_mode=1;
	if(throw_motor.state.rad>throw_start_rad)			//调整抛射大臂角度
	{
		throw_motor_target_vel=-0.5f;
		while(throw_motor.state.rad>throw_start_rad)
			vTaskDelay(3);
	}
	else if(throw_motor.state.rad<throw_start_rad)
	{
		throw_motor_target_vel=0.5f;
		while(throw_motor.state.rad<throw_start_rad)
			vTaskDelay(3);
	}
	throw_motor_target_vel=0.0f;

	if(jump_motor1.posVelEstimateGet.position<throw_height)			//调整抛射高度
	{
		jump_motor_target_vel=0.5f;
		while(jump_motor1.posVelEstimateGet.position<throw_height)
			vTaskDelay(3);
	}
	else if(jump_motor1.posVelEstimateGet.position>throw_height)
	{
		jump_motor_target_vel=-0.5f;
		while(jump_motor1.posVelEstimateGet.position>throw_height)
			vTaskDelay(3);
	}
	jump_motor_target_vel=0.0f;
	ActionFinished();
}

float GravityCompensation(float rad)
{
	UNUSED(rad);
	//TODO:计算重力补偿
	return 0.0f;
}

uint16_t SetSteeringEngineRAD180(float rad)
{
	const uint16_t base = 500;
	const float k = 1900 / PI;
	if (rad > PI)
		rad = PI;
	else if (rad < 0.0f)
		rad = 0.0f;
	return base + (uint16_t)(rad * k);
}

uint16_t SetSteeringEngineRAD270(float rad)
{
	const uint16_t base = 500;
	const float k = 1900 / (1.5*PI);
	if (rad > PI*1.5)
		rad = PI*1.5;
	else if (rad < 0.0f)
		rad = 0.0f;
	return base + (uint16_t)(rad * k);
}

void MotorTargetTrack(int target, int *ctrl_var, int rate)
{
	while (*ctrl_var!=target) // 复位到0
	{
		if (*ctrl_var > target+rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else if(*ctrl_var < target-rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var=target;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
}

void MotorTargetTrack_float(float target, float *ctrl_var, float rate)
{
	while (*ctrl_var!=target) // 复位到0
	{
		if (*ctrl_var > target+rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else if(*ctrl_var < target-rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var=target;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
}
