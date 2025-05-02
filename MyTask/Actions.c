#include "Action.h"
#include "Odrive.h"

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

// 电机控制接口变量
extern int32_t claw_motor_target_pos;
extern uint8_t push_motor_pos_mode;
extern float push_motor_target_pos;
extern float push_motor_target_vel;
extern int32_t pitch_motor_target_pos;
extern int32_t jump_motor_target_vel;
extern Motor2006Ex_t claw_motor;
//extern Motor3508Ex_t push_motor;
extern Motor3508Ex_t pitch_motor;
//extern Motor3508Ex_t jump_motor;
extern ODrive push_motor;

// 主体下降蓄力动作序列参数
float clutch_touch_angle = 100.0f;
float clutch_apart_angle = 20.0f;
float lock_angle_1 = 100.0f;
float lock_angle_2 = 0.0f;
float relese_angle_1 = 170.0f;
float relese_angle_2 = 70.0f;

// 运球任务基本参数配置
int32_t claw_touch_pos = -145313;
int32_t claw_detach_pos = -86816;
float push_gate_pos = -40000;
float push_stop_pos = -50000;
float launch_stop_pos = -12.965427;
float launch_final_pos = -15.8261852f;
int32_t pitch_hor_pos = -8000;
int32_t pitch_ver_pos = -95000;

float lv53_gate_dis = 270.0f;
float debug_value = 0.0f;
int32_t launch_velocity = -60.0f;
int32_t start_launch_time = 1100;

void ResetAction(void* param)	//上电时执行一次的动作，将机械结构件复位到正确的位置
{
	UNUSED(param);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)SetSteeringEngineRAD270(ANGLE2RAD(lock_angle_1)));
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(lock_angle_2)));
	pitch_motor_target_pos=pitch_hor_pos;
}

void TestAction(void *param)
{
	UNUSED(param);
	uint32_t step_ctrl=1;
	push_motor_pos_mode = 0;
	push_motor_target_vel = launch_velocity;
	while (push_motor.posVelEstimateGet.position > launch_stop_pos)		//到达发射终端位置
		vTaskDelay(pdMS_TO_TICKS(5));
	MotorTargetTrack_float(0, &push_motor_target_vel,0.3f, &step_ctrl);
	vTaskDelay(pdMS_TO_TICKS(500));
	push_motor_target_vel=10.0f;
	while (push_motor.posVelEstimateGet.position <0.0f)		//到达发射终端位置
		vTaskDelay(pdMS_TO_TICKS(5));
	push_motor_target_vel=0.0f;
}

void TestAction2(void *param)
{
	UNUSED(param);
	//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)SetSteeringEngineRAD270(ANGLE2RAD(lock_angle_1)));
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(lock_angle_2)));
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
}

// 准备运球动作序列
void ReadyDribbleAction(void *param)
{
	uint32_t *step_ctrl = (uint32_t *)param; // 方便使用遥控器强行终止该任务
	*step_ctrl = 1;
	MotorTargetTrack(claw_touch_pos, &claw_motor_target_pos, 100, step_ctrl);
	while (ABS(claw_motor.actual_pos - claw_motor_target_pos) > 1000 && (*step_ctrl))
		vTaskDelay(pdMS_TO_TICKS(5));
	MotorTargetTrack(pitch_ver_pos, &pitch_motor_target_pos, 100, step_ctrl);
}

// 准备跳跃投球动作序列
void ReadyLaunchAction(void *param)
{
	uint32_t *step_ctrl = (uint32_t *)param; // 方便使用遥控器强行终止该任务
	*step_ctrl = 1;
	MotorTargetTrack(pitch_hor_pos, &pitch_motor_target_pos, 100, step_ctrl);
}

// 开始运球动作序列
void DribbleAction(void *param)						// 运球动作序列
{ 
	uint32_t *setp_ctrl = (uint32_t *)param;
	*setp_ctrl = 1;
	push_motor_target_pos = push_stop_pos;
	while (push_motor.posVelEstimateGet.position > push_gate_pos) // 等待到达释放位置
		vTaskDelay(pdMS_TO_TICKS(5));
	claw_motor_target_pos = claw_detach_pos;

	while (lv53_sensor.distance < 300 && (*setp_ctrl)) // 等待球离开
		vTaskDelay(pdMS_TO_TICKS(5));
	while (lv53_sensor.distance > lv53_gate_dis && (*setp_ctrl)) // 等待球靠近
		vTaskDelay(pdMS_TO_TICKS(5));
	claw_motor_target_pos = claw_touch_pos;

	while (push_motor_target_pos < 0) // 回到起点
	{
		push_motor_target_pos = push_motor_target_pos + 100;
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

// 开始跳跃投球动作序列
void LaunchAction(void *param)
{
	uint32_t step_ctrl=1;
	//MotorTargetTrack(0, &claw_motor_target_pos, 100, &step_ctrl);
	pitch_motor_target_pos=0;
	// TODO:使用v=f(x,e_v)，使PID控制器跟踪目标速度
	// TODO:到达某个数字后，松开夹爪
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)SetSteeringEngineRAD270(ANGLE2RAD(clutch_apart_angle)));
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)SetSteeringEngineRAD270(ANGLE2RAD(relese_angle_1)));
	vTaskDelay(pdMS_TO_TICKS(500));
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(relese_angle_2)));
	vTaskDelay(pdMS_TO_TICKS(start_launch_time));		//确保主体开始上行

	Action_t action={.action_cb=ClutchLockAction,.param=NULL};
	xQueueSend(copilot_action_queue,&action,0);		//执行检测速度曲线，使离合器准备耦合的动作
	//TODO；根据运动学计算，此时需要进行投射动作
	push_motor_pos_mode = 0;
	push_motor_target_vel = launch_velocity;
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	while (push_motor.posVelEstimateGet.position > launch_stop_pos)		//到达发射终端位置
		vTaskDelay(pdMS_TO_TICKS(5));
	push_motor_target_vel=0;
	vTaskDelay(pdMS_TO_TICKS(500));
	push_motor_target_pos = push_motor.posVelEstimateGet.position;
	push_motor_pos_mode = 1;
	MotorTargetTrack_float(0, &push_motor_target_pos, 50, &step_ctrl);
}

void ClutchLockAction(void* param)
{
	int32_t speed[5] = {0,10};
	int32_t sum;
	uint8_t p_data;
	uint32_t last_dis=lv53_sensor_chass.distance;
	
	do
	{
		vTaskDelay(pdMS_TO_TICKS(20));	//Lv53数据更新时间
		speed[p_data] = chassis_v;
		p_data = (p_data + 1) % 5;

		sum = 0;
		for (uint8_t i = 0; i < 5; i++)
			sum = sum + speed[i];
		} while (sum>0);					//一直等待直到速度为负值（撞击桅杆顶端之后）
	
	do
	{
		vTaskDelay(pdMS_TO_TICKS(20));
		speed[p_data] = chassis_v;
		p_data = (p_data + 1) % 5;

		sum = 0;
		for (uint8_t i = 0; i < 5; i++)
			sum = sum + speed[i];
	} while (sum<0);					//一直等待直到速度为正值（达到最低点之后开始上升）
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
	//下降到最低点，离合器耦合
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(clutch_touch_angle)));	//到达底端后离合器耦合
	Action_t action={.action_cb=MainBodyDeclineAction};
	xQueueSend(action_queue,&action,0);
}

int16_t decline_threshold=100;

// 主体下降动作序列
void MainBodyDeclineAction(void *param)
{
	int32_t speed[10] = {0};
	int32_t sum;
	uint8_t p_data;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(clutch_touch_angle))); // 离合器耦合
	//TODO:舵机拉起机械锁定装置
	vTaskDelay(pdMS_TO_TICKS(200));	//确保离合器耦合后，开始加速
	jump_motor_target_vel = 4000;
	while (lv53_sensor_chass.distance>decline_threshold)
		vTaskDelay(pdMS_TO_TICKS(5));
	jump_motor_target_vel = 1000;
	do
	{
		vTaskDelay(pdMS_TO_TICKS(10));
		speed[p_data] = 0;	//Debug
		p_data = (p_data + 1) % 10;

		sum = 0;
		for (uint8_t i = 0; i < 10; i++)
			sum = sum + ABS(speed[i]);
	} while (sum); // 等待直到连续10个采样点的速度数据均为0
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(lock_angle_1))); // 锁定主体
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)SetSteeringEngineRAD270(ANGLE2RAD(lock_angle_2))); // 锁定主体
	vTaskDelay(pdMS_TO_TICKS(500));
	jump_motor_target_vel = 0;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)SetSteeringEngineRAD180(ANGLE2RAD(clutch_apart_angle))); // 离合器
	// TODO:控制舵机锁死
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

uint32_t MotorTargetTrack(int target, int *ctrl_var, int rate, uint32_t *exit_ctrl)
{
	while (ABS(target - *ctrl_var) > TARGET_TRACK_UPDATE_TIME * rate && (*exit_ctrl)) // 复位到0
	{
		if (*ctrl_var > target)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
	return 0;
}

float MotorTargetTrack_float(float target, float *ctrl_var, float rate, uint32_t *exit_ctrl)
{
	while (ABS(target - *ctrl_var) > TARGET_TRACK_UPDATE_TIME * rate && (*exit_ctrl)) // 复位到0
	{
		if (*ctrl_var > target)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
	*ctrl_var=target;	//确保到达目标
	return 0;
}