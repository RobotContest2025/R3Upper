#include "Action.h"
#include "Odrive.h"
#include "RobStride2.h"

extern TIM_HandleTypeDef htim1;
extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;
extern QueueHandle_t action_deal_mutex;

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
extern RobStride_t pitch_motor;
extern float pitch_motor_actual_pos;

extern uint8_t jump_motor_vel_mode;
extern uint8_t pitch_motor_pos_mode;
extern int32_t claw_motor_target_pos;
extern float push_motor_target_vel;
extern float pitch_motor_target_pos;
extern float jump_motor_target_vel;
extern float jump_motor_target_cur;
extern float pitch_motor_target_cur;

float lv53_gate_dis = 270.0f;
float debug_value = 0.0f;
int32_t launch_velocity = -60.0f;
int32_t start_launch_time = 1100;


void ResetAction(void* param)	//上电时执行一次的动作，将机械结构件复位到正确的位置
{
	UNUSED(param);
	MotorTargetTrack_float(0.0f, &pitch_motor_target_pos,0.001f);
	
	task_exit=0;
	vTaskDelete(NULL);
}

float pitch_lock_cur=-5.0f;		//俯仰电机锁定电流

float launch_gate_pos=165071.0f;
float launch_vel=2000.0f;
float zero_rate=50.0f;
void TestAction(void *param)
{
	UNUSED(param);

	pitch_motor_target_cur=pitch_lock_cur;
	
	push_motor_target_vel=launch_vel;
	while(push_motor.actual_pos<launch_gate_pos)
		vTaskDelay(5);
	MotorTargetTrack_float(0.0f, &push_motor_target_vel, zero_rate);
	pitch_motor_target_cur=0.0f;	//释放俯仰电机
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;
	
	//pitch_motor_pos_mode=1;
	
	task_exit=0;
	vTaskDelete(NULL);
}

void LaunchAction(void *param)
{
	UNUSED(param);
	pitch_motor_target_cur=pitch_lock_cur;
	push_motor_target_vel=launch_vel;
	while(push_motor.actual_pos<launch_gate_pos)
		vTaskDelay(5);
	MotorTargetTrack_float(0.0f, &push_motor_target_vel, zero_rate);
	pitch_motor_target_cur=0.0f;
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;
}


float jump_vel=-20.0f;
float jump_cur=-60.0f;
float jump_stop_pos=-5.0f;
float break_cur=60.0f;


void TestAction2(void *param)
{
	Action_t action={.action_cb=LaunchAction};
	UNUSED(param);
	float reset_offset=0.0f;
	float vel_record[16]={1.0f};
	int record_point=0;
	float sum=0.0f;
	
	jump_motor_target_vel=3.0f;
	jump_motor_vel_mode=1;
	vTaskDelay(2000);
	do
	{
		vTaskDelay(50);
		vel_record[record_point]=jump_motor1.posVelEstimateGet.velocity;
		record_point=(record_point+1)%16;
		sum=0.0f;
		for(int i=0;i<16;i++)
			sum+=ABS(vel_record[i]);
		sum=sum/16.0f;
	}while(sum>0.01f);
	reset_offset=jump_motor1.posVelEstimateGet.position;		//到达最低端，并记录最低端的位置
	
	jump_motor_target_cur=jump_cur;
	jump_motor_vel_mode=0;
	jump_motor_target_vel=0.0f;
	while((jump_motor1.posVelEstimateGet.position>jump_stop_pos+reset_offset)&&(jump_motor1.posVelEstimateGet.velocity>jump_vel))//等待直到加速到期望速度或者到达刹车点
		vTaskDelay(5);
	
	xQueueSend(copilot_action_queue,&action,0);		//执行发射动作
	
	jump_motor_target_vel=jump_vel;	//切入速度闭环模式保持速度
	jump_motor_vel_mode=1;
	jump_motor_target_cur=0.0f;
	while(jump_motor1.posVelEstimateGet.position>jump_stop_pos+reset_offset)	//等待直到到达刹车点
		vTaskDelay(5);
	
	jump_motor_target_cur=break_cur;		//刹车
	jump_motor_vel_mode=0;
	while(jump_motor1.posVelEstimateGet.velocity<0.0f)	//刹车完成
		vTaskDelay(5);
	jump_motor_target_cur=0.0f;
	jump_motor_target_vel=0.0f;
	jump_motor_vel_mode=1;
	
	task_exit=0;
	vTaskDelete(NULL);
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
