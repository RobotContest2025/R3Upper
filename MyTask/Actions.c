#include "Action.h"
#include "Odrive.h"
#include "RobStride2.h"
#include "tim.h"
#include "MoveControl.h"
#include "RemoteTask.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;
extern QueueHandle_t action_semaphore;

extern Motor2006Ex_t claw_motor;
extern Motor3508Ex_t push_motor;
extern ODrive jump_motor1;
extern ODrive jump_motor2;
extern RobStride_t throw_motor;

extern uint8_t throw_motor_vel_mode;
extern uint8_t jump_motor_vel_mode;
extern uint8_t jump_motor_enable;
extern int32_t claw_motor_target_pos;
extern float throw_motor_target_vel;
extern float throw_motor_target_torque;
extern float jump_motor_target_vel;
extern float jump_motor_target_cur; // 这里的电流值送入Odrive中后乘了25
extern float lift_motor1_target_vel;
extern float lift_motor2_target_vel;
extern float throw_motor_target_rad;
extern int16_t max_2006_cur;

extern RobotMode robot_state;
float remote_rocker4;
extern ChassisCtrl_t chassis;
extern JY61P_AccSensor_t jy61p;
extern RobotState_t robot;
extern PID2 jump_motor_pid;

int lv53_ball_dis = 0;
float lv53_cur_dis = 0.0f;
float lv53_gate_dis = 270.0f;
float debug_value = 0.0f;
float reset_height = 0.935f;
int32_t launch_velocity = -60.0f;
int32_t start_launch_time = 1100;

extern float actual_gn;

float lower_height = 0.20f;
float reset_vel = -3.0f;
float throw_motor_reset_rad = 0.3f;
void ResetAction(void *param) // 上电时执行一次的动作，将机械结构件复位到正确的位置
{
	UNUSED(param);
	jump_motor_vel_mode = 1;
	jump_motor_target_vel = 0.0f;
	throw_motor_vel_mode = 1;
	throw_motor_target_vel = 0.0f;
	throw_motor_target_torque = 0.0f;

	chassis.cmd = CMD_MODE_UNLOCK_CHASSIS;

	jump_motor_target_vel = 2.0f;
	while (lv53_cur_dis < reset_height)
		vTaskDelay(3);
	jump_motor_target_vel = 0.0f;

	chassis.cmd = CMD_MODE_LOCK_CHASSIS;

	throw_motor_target_rad = throw_motor.state.rad;
	throw_motor_vel_mode = 2;
	MotorTargetTrack_float(throw_motor_reset_rad, &throw_motor_target_rad, 0.001f);

	jump_motor_target_vel = reset_vel;
	while (lv53_cur_dis > lower_height + 0.05f)
		vTaskDelay(3);
	jump_motor_target_vel = 0.0f;
	jump_motor_vel_mode = 0;
	jump_motor_target_cur = -0.05f;
	vTaskDelay(700); // 等待完全与下底盘接触
	jump_motor_target_cur = 0.0f;
	jump_motor_vel_mode = 1;
	chassis.cmd = CMD_MODE_MOVE;
	ActionFinished();
}

void DownChassis(void *param)
{
	UNUSED(param);
	chassis.cmd = CMD_MODE_LAUNCH;
	vTaskDelay(100);
	ActionFinished();
}

float lower_vel = -2.0f;
float locked_gate_cur = -0.45f;
float height_offset = 0.0f;
float throw_start_rad = 2.2f;
float throw_break_torque = 50.0f;
float throw_detach_rad = 2.8f;
float throw_break_rad = 3.0f;
float claw_touch_pos = 0.0f;
int32_t claw_detach_pos = 150000;

float throw_target_vel = 1.0f;

uint8_t chassis_locked = 1;
// LaunchDots_t dots = {.distance = 2.8f, .jump_motor_iq = 0.6f, .throw_motor_torque = 30.0f, .detach_angle = 120.0f, .break_angle = 150.0f};
LaunchDots_t dots = {.distance = 2.8f, .jump_motor_iq = 0.65f, .throw_motor_torque = -5.0f, .detach_angle = 60.0f, .break_angle = 20.0f};
float finished_throw_target_height = 6.0f;
float finished_throw_target_height_sensor = 0.73f;

void ReadyBackThrowAction(void *param)
{
	throw_motor_target_rad = throw_motor.state.rad;
    throw_motor_target_torque = 0;
	throw_motor_vel_mode = 2;
	MotorTargetTrack_float(throw_start_rad, &throw_motor_target_rad, 0.002f);

	jump_motor_vel_mode = 1;
	jump_motor_target_vel = lower_vel;
	chassis.cmd = CMD_MODE_LOCK_CHASSIS; // 发送指令让底盘锁定
	while (lv53_cur_dis > lower_height)
		vTaskDelay(3);
	jump_motor_vel_mode = 0;
	jump_motor_target_cur = -0.1f;
	vTaskDelay(1000);
	jump_motor_vel_mode = 1;
	jump_motor_target_cur = 0.0f;
	jump_motor_target_vel = 0.0f;
	//dots = CalculateLaunchParam(robot.Distance);
	ActionFinished();
}

void BackThrowAction(void *param)
{
	UNUSED(param);
	jump_motor_vel_mode = 0;
	jump_motor_target_vel = 0.0f;
    throw_motor_vel_mode = 0;
	// TODO:略微旋转电机使恒力弹簧收紧，之后等待
	jump_motor_target_cur = 0.2f; // 收紧恒力弹簧
	max_2006_cur = 10000;
	chassis.cmd = CMD_MODE_LAUNCH;
	vTaskDelay(200);
	chassis.cmd = CMD_MODE_UNLOCK_CHASSIS;
	vTaskDelay(300);
	throw_motor_vel_mode = 0;
	jump_motor_target_cur = dots.jump_motor_iq;
	vTaskDelay(25);
	throw_motor_target_torque = dots.throw_motor_torque;
	while (throw_motor.state.rad > ANGLE2RAD(dots.detach_angle)) // 加速到了分离点
	{
		vTaskDelay(3);
	}
	claw_motor_target_pos = claw_detach_pos; // 松开夹爪并且关闭投射电机
	throw_motor_target_torque = 0.0f;
	// jump_motor_target_cur = 0.2f;

	while (throw_motor.state.rad > ANGLE2RAD(dots.break_angle) && throw_motor.state.omega < 0.0f) // 到达了刹车点或者投射电机速度在到达刹车点之前归零则继续执行
		vTaskDelay(3);
	throw_motor_target_torque = throw_break_torque;
	jump_motor_target_cur = 0.0f;
	jump_motor_enable = 0;
	jump_motor_vel_mode = 0;
	jump_motor_target_vel = 0.0f;

	uint8_t flag1 = 1;
	while (throw_motor.state.omega < 0.0f && jump_motor1.posVelEstimateGet.velocity > 0.1f) // 等待大臂刹车完成或者速度足够小（尚未撞击桅杆速度就已经归零）
		vTaskDelay(3);
	if (jump_motor1.posVelEstimateGet.velocity <= 0.1f) // 提前自己停下了
		flag1 = 0;

	throw_motor_target_torque = 0.0f;
	throw_motor_vel_mode = 1;
	throw_motor_target_vel = 0.0f;
	if (flag1)
	{
		while (jump_motor1.posVelEstimateGet.velocity > 0.1f) // 等待直到直到撞击完桅杆
			vTaskDelay(3);

		jump_motor_enable = 1;
		jump_motor_target_cur = 0.8f;
		while (jump_motor1.posVelEstimateGet.velocity < 0.0f) // 上底盘开始刹车
			vTaskDelay(3);
	}

	jump_motor_target_cur = 0.0f;
	jump_motor_vel_mode = 1;
	jump_motor_enable = 1;

	chassis.cmd = CMD_MODE_LOCK_CHASSIS; // 发送指令准备锁定上下底盘

	if (lv53_cur_dis > finished_throw_target_height_sensor) // 移动到确定的位置使大臂归位
	{
		jump_motor_target_vel = -2.0f;
		while (lv53_cur_dis > finished_throw_target_height_sensor)
			vTaskDelay(3);
	}
	else
	{
		jump_motor_target_vel = 2.0f;
		while (lv53_cur_dis < finished_throw_target_height_sensor)
			vTaskDelay(3);
	}
	jump_motor_target_vel = 0.0f;

	throw_motor_vel_mode = 1;
	throw_motor_target_vel = 0.5f;
	while (throw_motor.state.rad < throw_start_rad) // 大臂归位
		vTaskDelay(3);
	throw_motor_target_vel = 0.0f;

	chassis.cmd = CMD_MODE_MOVE;

	jump_motor_target_vel = -2.0f;
	while (lv53_cur_dis > lower_height + 0.01f) // 小底盘下降
		vTaskDelay(3);
	jump_motor_vel_mode = 0;
	jump_motor_target_cur = -0.1f;
	vTaskDelay(1000); // 等待完全锁定
	jump_motor_target_cur = 0.0f;
	jump_motor_target_vel = 0.0f;

	claw_motor_target_pos = 0;
	max_2006_cur = 3000;
	ActionFinished();
}

uint8_t claw_next_step = 1;
void DetachClawAction(void *param)
{
	claw_next_step = 1;
	claw_motor_target_pos = claw_detach_pos;
	vTaskDelay(100);
	ActionFinished();
}

float dribble2_height = 0.527f; // 运球时大臂的高度
float LossE_nergy_Rate = 0.808,
	  Arm_Length = 0.68,
	  dribble_angle = 10; // 运球动作水平面对称角度 角度制
void ReadyDribbleAction2(void *param)
{ 
	UNUSED(param);
	jump_motor_vel_mode = 1;
	jump_motor_target_cur = 0.0f;
	throw_motor_vel_mode = 2;
	throw_motor_target_rad = throw_motor.state.rad;
	throw_motor_target_torque = 0.0f;

	float Half_Chord_Length = sinf(ANGLE2RAD(dribble_angle)) * Arm_Length;
	float Release_Height = Half_Chord_Length * 2 / (1 - LossE_nergy_Rate);
	dribble2_height = Release_Height - Half_Chord_Length - 0.5849f; // 添加实际传感器应追踪的高度

	chassis.cmd = CMD_MODE_UNLOCK_CHASSIS;
	vTaskDelay(300);

	if (throw_motor.state.rad < ANGLE2RAD(13)){
        jump_motor_enable = 1;
        jump_motor_vel_mode = 1;
        jump_motor_target_vel = 1.5f;
        do{
            vTaskDelay(3);//必须先延时
            if(lv53_cur_dis > 0.66)
                jump_motor_target_vel = 0.f;
            if(throw_motor.state.rad < ANGLE2RAD(-13))
                throw_motor_target_rad = TargetSlope(ANGLE2RAD(-13), throw_motor_target_rad, 0.003);
        }while (lv53_cur_dis < 0.66 || throw_motor.state.rad < ANGLE2RAD(-13));
        MotorTargetTrack_float(ANGLE2RAD(13), &throw_motor_target_rad, 0.001f);
    }
    if(throw_motor.state.rad > ANGLE2RAD(110)){//如果太靠下
        throw_motor_target_rad = throw_motor.state.rad;
        throw_motor_vel_mode = 2;
        MotorTargetTrack_float(ANGLE2RAD(110), &throw_motor_target_rad, 0.001f);
    }

	if (lv53_cur_dis > dribble2_height - 0.03) // 调整运球起始大臂高度
	{
		jump_motor_target_vel = -2.0f;
		while (lv53_cur_dis > dribble2_height - 0.03){
            throw_motor_target_rad = TargetSlope(ANGLE2RAD(90.0f - dribble_angle), throw_motor_target_rad, 0.003);
			vTaskDelay(3);
        }
		jump_motor_target_vel = 1.0f;
		while (lv53_cur_dis < dribble2_height){
            throw_motor_target_rad = TargetSlope(ANGLE2RAD(90.0f - dribble_angle), throw_motor_target_rad, 0.003);
			vTaskDelay(3);
        }
	}
	else if (lv53_cur_dis < dribble2_height - 0.03)
	{
		jump_motor_target_vel = 2.0f;
		while (lv53_cur_dis < dribble2_height - 0.03){
            throw_motor_target_rad = TargetSlope(ANGLE2RAD(90.0f - dribble_angle), throw_motor_target_rad, 0.003);
			vTaskDelay(3);
        }
		jump_motor_target_vel = 1.0f;
		while (lv53_cur_dis < dribble2_height){
            throw_motor_target_rad = TargetSlope(ANGLE2RAD(90.0f - dribble_angle), throw_motor_target_rad, 0.003);
			vTaskDelay(3);
        }
	}
	jump_motor_target_vel = 0.0f;
    MotorTargetTrack_float(ANGLE2RAD(90.0f - dribble_angle), &throw_motor_target_rad, 0.001f);

	ActionFinished();
}

float dribble_rate = 0.0014f;
float dribble_angle_Bias = 4;
void DribbleAction2(void *param)
{
	UNUSED(param);
	claw_motor_target_pos = claw_detach_pos;
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) // 等待球离开
		vTaskDelay(5);
	vTaskDelay(100); // 与球分开一定距离

	throw_motor_vel_mode = 2;
	MotorTargetTrack_float(ANGLE2RAD(90.0f + dribble_angle + dribble_angle_Bias), &throw_motor_target_rad, dribble_rate);

	do{
		vTaskDelay(3);
	} while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);

	claw_motor_target_pos = 0;

	ActionFinished();
}

float jumpthrow_start_rad = 0.52f;
float jump_motor_offset = 0.0f;
float jump_arm_reset_rate = 0.002f;
void ReadyJumpAction(void *param)
{
	UNUSED(param);
	jump_motor_vel_mode = 1;
	jump_motor_target_vel = 0.0f;

	chassis.cmd = CMD_MODE_LOCK_CHASSIS; // 发送指令让底盘锁定

	throw_motor_vel_mode = 2;
	throw_motor_target_rad = throw_motor.state.rad;
	throw_motor_target_torque = 0.0f;

	if (throw_motor.state.rad < ANGLE2RAD(13)){
        jump_motor_enable = 1;
        jump_motor_vel_mode = 1;
        jump_motor_target_vel = 1.5f;
        do{
            vTaskDelay(3);//必须先延时
            if(lv53_cur_dis > 0.66)
                jump_motor_target_vel = 0.f;
            if(throw_motor.state.rad < ANGLE2RAD(-13))
                throw_motor_target_rad = TargetSlope(ANGLE2RAD(-13), throw_motor_target_rad, 0.003);
        }while (lv53_cur_dis < 0.66 || throw_motor.state.rad < ANGLE2RAD(-13));
        MotorTargetTrack_float(ANGLE2RAD(13), &throw_motor_target_rad, 0.001f);
    }
    if(throw_motor.state.rad > ANGLE2RAD(110)){//如果太靠下
        throw_motor_target_rad = throw_motor.state.rad;
        throw_motor_vel_mode = 2;
        MotorTargetTrack_float(ANGLE2RAD(110), &throw_motor_target_rad, 0.001f);
    }
	if (lv53_cur_dis > lower_height) // 调整跳跃起始大臂高度
	{
		jump_motor_target_vel = -2;
		while (lv53_cur_dis > lower_height){
			vTaskDelay(3);
            throw_motor_target_rad = TargetSlope(jumpthrow_start_rad, throw_motor_target_rad, 0.006);
        }
	}
	jump_motor_target_vel = 0.0f;

	MotorTargetTrack_float(jumpthrow_start_rad, &throw_motor_target_rad, jump_arm_reset_rate);

	jump_motor_vel_mode = 0;
	jump_motor_target_cur = -0.1f;
	vTaskDelay(1000);

	jump_motor_vel_mode = 1;
	jump_motor_target_cur = 0.0f;
	jump_motor_target_vel = 0.0f;
	ActionFinished();
}

float jump_start_cur = 2.1f;
uint32_t jump_delay = 30;
uint32_t down_delay = 200;
uint32_t detach_delay = 100;
float jump_stop_dis = 0.8f;
float jump_throw_vel = -2.3f;
float jump_throw_back_vel = 3.0f;
float jump_throw_stop_rad = 0.1f;
float jump_break_cur = 1;
float jump_stop_height = 0.95f;
float jump_motor_stop_pos = 8;

void JumpAction(void *param)
{
	UNUSED(param);
	throw_motor_vel_mode = 2;
	throw_motor_target_torque = 0.0f;
	jump_motor_enable = 1;
	jump_motor_vel_mode = 0;
	jump_motor_target_vel = 0.0f;

	jump_motor_target_cur = 0.04f; // 收紧恒力弹簧
	chassis.cmd = CMD_MODE_JUMP;   // 底盘转到跳跃模式
	vTaskDelay(2500);			   // 等待底盘完成模式转换
	jump_motor_offset = jump_motor1.posVelEstimateGet.position;
	jump_motor_target_cur = jump_start_cur;
	while (jump_motor1.posVelEstimateGet.position - jump_motor_offset < jump_motor_stop_pos)
		vTaskDelay(3);
	jump_motor_enable = 0;
	vTaskDelay(jump_delay);
	throw_motor_vel_mode = 1;
	throw_motor_target_vel = jump_throw_vel;
	while (throw_motor.state.rad > jump_throw_stop_rad)
		vTaskDelay(3);
	claw_motor_target_pos = claw_detach_pos;
	vTaskDelay(detach_delay);
	throw_motor_target_vel = jump_throw_back_vel;
	vTaskDelay(down_delay); // 延时等待机器人落地

	while (jump_motor1.posVelEstimateGet.position - jump_motor_offset > jump_motor_stop_pos)
		vTaskDelay(3);

	throw_motor_vel_mode = 2;
	throw_motor_target_rad = throw_motor.state.rad;

	jump_motor_enable = 1;
	jump_motor_target_vel = 0.0f;
	jump_motor_target_cur = 0.0f;
	jump_motor_vel_mode=1;
	vTaskDelay(100);
	float a=0.5f*jump_motor1.posVelEstimateGet.velocity*jump_motor1.posVelEstimateGet.velocity/(jump_motor1.posVelEstimateGet.position-jump_motor_offset);
	while(jump_motor1.posVelEstimateGet.velocity<0.0f)	//等待直到机器人刹车完成
	{
		jump_motor_target_vel=-sqrt(2.0f*a*(jump_motor1.posVelEstimateGet.position-jump_motor_offset));
		vTaskDelay(3);
	}
	jump_motor_vel_mode=0;
	jump_motor_target_cur = -0.2;
	while(jump_motor1.posVelEstimateGet.position > jump_motor_offset+0.5f)//防止电机没着地
		vTaskDelay(3);

	/*jump_motor_target_vel = 0.0f;
	jump_motor_target_cur = jump_break_cur;
	while (jump_motor1.posVelEstimateGet.velocity < 0.0f) // 等待直到机器人刹车完成
		vTaskDelay(3);
	jump_motor_target_cur = -0.2;
	while (jump_motor1.posVelEstimateGet.position > jump_motor_offset + 0.5) // 防止电机没着地
		vTaskDelay(3);*/

	jump_motor_target_cur = 0.0f;
	jump_motor_vel_mode = 1;
	jump_motor_target_vel = 0.0f;
	vTaskDelay(500);
	chassis.cmd = CMD_MODE_MOVE; // 给底盘发送指令要求转换成移动模式
	vTaskDelay(2000);
	ActionFinished();
}

float pickup_rad = 2.5f;
float pickup_height = 0.2f;
float pickup_start_height = 0.6f;
void PickUpBallAction(void *param)
{
	UNUSED(param);
	throw_motor_target_rad = throw_motor.state.rad;
	throw_motor_vel_mode = 2;
    throw_motor_target_torque = 0;
    claw_motor_target_pos = claw_detach_pos;

	if (throw_motor.state.rad < ANGLE2RAD(13)){
        jump_motor_enable = 1;
        jump_motor_vel_mode = 1;
        jump_motor_target_vel = 1.5f;
        do{
            vTaskDelay(3);//必须先延时
            if(lv53_cur_dis > 0.66)
                jump_motor_target_vel = 0.f;
            if(throw_motor.state.rad < ANGLE2RAD(-13))
                throw_motor_target_rad = TargetSlope(ANGLE2RAD(-13), throw_motor_target_rad, 0.003);
        }while (lv53_cur_dis < 0.66 || throw_motor.state.rad < ANGLE2RAD(-13));
        MotorTargetTrack_float(ANGLE2RAD(13), &throw_motor_target_rad, 0.001f);
    }

    if(throw_motor.state.rad > ANGLE2RAD(110)){//如果太靠下
        throw_motor_target_rad = throw_motor.state.rad;
        throw_motor_vel_mode = 2;
        MotorTargetTrack_float(ANGLE2RAD(110), &throw_motor_target_rad, 0.001f);
    }

	if (lv53_cur_dis > lower_height) // 调整捡球起始大臂高度
	{
		jump_motor_target_vel = -2.0f;
		while (lv53_cur_dis > lower_height){
			vTaskDelay(3);
            throw_motor_target_rad = TargetSlope(pickup_rad, throw_motor_target_rad, 0.003);
        }
	}
	else if (lv53_cur_dis < lower_height)
	{
		jump_motor_target_vel = 2.0f;
		while (lv53_cur_dis < lower_height){
			vTaskDelay(3);
            throw_motor_target_rad = TargetSlope(pickup_rad, throw_motor_target_rad, 0.003);
        }
	}
	jump_motor_target_vel = 0.0f;

    MotorTargetTrack_float(pickup_rad, &throw_motor_target_rad, 0.001f);
    
    do{//对着球直接冲，如果没捡到就再执行此动作，所有前置准备语句应该会瞬间完成
		vTaskDelay(3);
	} while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);

    vTaskDelay(200);

	claw_motor_target_pos = 0;

	ActionFinished();
}

float cur_exp_vel;
float height_err;

float defend_vel_limit=20.0f;
float defend_rad = 0.3f;
float defend_exp_height = 0.0f;
float exp_pos_k = 0.4f;			 // 归一化后的摇杆数据到高度的比例关系
float exp_pos_b = 0.6f;			 // 上底盘默认高度
float height_Kp = 40.0f;			 // 上底盘高度的位置环比例系数
float rise_acc_limit = 50.0f;	 // 上地盘上升的加速度限制（单位：r/s^2）
float decline_acc_limit = -30.0f; // 上地盘下降的加速度限制
uint8_t exit_defend;
void DefendAction(void *param)
{
	UNUSED(param);
	chassis.cmd=CMD_MODE_UNLOCK_CHASSIS;
	throw_motor_target_rad = throw_motor.state.rad;
    throw_motor_target_torque = 0;
	throw_motor_vel_mode = 2;
	jump_motor_enable = 1;
	jump_motor_vel_mode = 1;
	if (lv53_cur_dis > exp_pos_b)
	{
		jump_motor_target_vel = -2.0f;
		while (lv53_cur_dis > exp_pos_b)
			vTaskDelay(3);
	}
	else if (lv53_cur_dis < exp_pos_b)
	{
		jump_motor_target_vel = 2.0f;
		while (lv53_cur_dis < exp_pos_b)
			vTaskDelay(3);
	}
	jump_motor_target_vel = 0.0f;

	MotorTargetTrack_float(defend_rad, &throw_motor_target_rad, 0.001f);

	float last_exp_vel = 0.0f;
	while (exit_defend == 0) // 正常情况下动作会阻塞在这里直到下一个动作打断当前动作（当前动作应被配置为可打断的）
	{
		defend_exp_height = exp_pos_k * remote_rocker4 + exp_pos_b; // 根据遥控器摇杆4的位置计算期望高度

		height_err = defend_exp_height - lv53_cur_dis;
		cur_exp_vel = height_err * height_Kp; // 计算期望速度
		limit(cur_exp_vel, defend_vel_limit, -defend_vel_limit);			// 限制期望速度

		float d_vel = cur_exp_vel - last_exp_vel;						 // 计算期望速度的变化量
		limit(d_vel, rise_acc_limit * 0.01f, decline_acc_limit * 0.01f); // 限制加速度防止恒力弹簧损坏

		jump_motor_target_vel = last_exp_vel + d_vel;
		last_exp_vel = jump_motor_target_vel; // 记录上一次的期望速度
		vTaskDelay(10);
	}
	ActionFinished();
}

LaunchDots_t CalculateLaunchParam(float distance)
{
	LaunchDots_t temp = {.distance = 2.8f, .jump_motor_iq = 0.65f, .throw_motor_torque = -30.0f, .detach_angle = 60.0f, .break_angle = 20.0f};
	return temp;
}

float friction_compen = 0.3f; // 摩擦力补偿
float arm_torque_compen_90 = 3.13f;
float ball_torque_compen_90 = 5.13f - 3.13f; // 球产生的力矩
float GravityTorqueCompensation(float rad, float omega, float gn, uint8_t has_ball)
{
	rad = rad - 3.14159265f;	
	float torque_feedforward = 0.0f;

	if (omega > 0.05f) // 摩擦力补偿
		torque_feedforward = friction_compen;
	else if (omega < -0.05f)
		torque_feedforward = -friction_compen;

	if (has_ball) // 重力补偿
		torque_feedforward = torque_feedforward + ball_torque_compen_90 * sin(rad);
	torque_feedforward = torque_feedforward + arm_torque_compen_90 * sin(rad);

	return torque_feedforward * gn;
}

uint8_t next_measure = 1;
uint32_t dt_ms;
void MeasureTask(void *param) // 重力补偿参数测量任务，给next_measure置0可以执行下一步
{
	next_measure = 1;
	while (next_measure)
		vTaskDelay(200);
	next_measure = 1;

	throw_motor_vel_mode = 1;
	throw_motor_target_torque = 0.0f;
	if (throw_motor.state.rad > 1.570796325f) // 调整抛射大臂角度到90度
	{
		throw_motor_target_vel = -0.1f;
		while (throw_motor.state.rad > 1.570796325f)
			vTaskDelay(3);
	}
	else if (throw_motor.state.rad < 1.570796325f)
	{
		throw_motor_target_vel = 0.1f;
		while (throw_motor.state.rad < 1.570796325f)
			vTaskDelay(3);
	}
	throw_motor_target_vel = 0.0f;

	while (next_measure)
		vTaskDelay(200);
	next_measure = 1;

	throw_motor_vel_mode = 0;
	throw_motor_target_vel = 0;
	while (next_measure) // 验证重力和摩擦力补偿的成果
	{
		throw_motor_target_torque = GravityTorqueCompensation(throw_motor.state.rad, throw_motor.state.omega, 0.0f, 0);
		vTaskDelay(3);
	}
	throw_motor_target_torque = 0.0f;
	throw_motor_vel_mode = 1;
	throw_motor_target_vel = 0.0f;
	ActionFinished();
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
	const float k = 1900 / (1.5 * PI);
	if (rad > PI * 1.5)
		rad = PI * 1.5;
	else if (rad < 0.0f)
		rad = 0.0f;
	return base + (uint16_t)(rad * k);
}

// 目标值，当前值，最大增加速度，最大减小速度
float TargetSlope(float target, float current, float max_increase)
{
	if (target - current > max_increase)		// 如果目标值大于当前值加上最大改变量
		return current + max_increase;			// 返回当前值加上最大改变量
	else if (target - current < -max_increase) // 如果目标值小于当前值减去最大改变量
		return current - max_increase;			// 返回当前值减去最大改变量
	else
		return target; // 否则返回目标值
}

void MotorTargetTrack(int target, int *ctrl_var, int rate)
{
	while (*ctrl_var != target) // 复位到0
	{
		if (*ctrl_var > target + rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else if (*ctrl_var < target - rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var = target;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
}

void MotorTargetTrack_float(float target, float *ctrl_var, float rate)
{
	while (*ctrl_var != target) // 复位到0
	{
		if (*ctrl_var > target + rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var - rate * TARGET_TRACK_UPDATE_TIME;
		else if (*ctrl_var < target - rate * TARGET_TRACK_UPDATE_TIME)
			*ctrl_var = *ctrl_var + rate * TARGET_TRACK_UPDATE_TIME;
		else
			*ctrl_var = target;
		vTaskDelay(pdMS_TO_TICKS(TARGET_TRACK_UPDATE_TIME));
	}
}

