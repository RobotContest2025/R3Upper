#include "motorControl.h"
#include "PID_old.h"
#include "Odrive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart5;

extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;


Motor2006Ex_t claw_motor = {.hcan = &hcan2, .ID = 0x201};
//Motor3508Ex_t push_motor = {.hcan = &hcan2, .ID = 0x202};
Motor3508Ex_t pitch_motor = {.hcan = &hcan1, .ID = 0x201};
//Motor3508Ex_t jump_motor = {.hcan = &hcan1, .ID = 0x203};
ODrive push_motor={.hcan=&hcan1,.motorID=0x010};
PID2 push_motor_pid;


uint8_t push_motor_pos_mode=1;
int32_t claw_motor_target_pos = 0;
float push_motor_target_pos = 0;
float push_motor_target_vel = 0;
int32_t pitch_motor_target_pos=0;
int32_t jump_motor_target_vel=0;

// 电机PID闭环控制任务
int16_t canSendBuf[4] = {0};
int16_t canSendBuf2[4] = {0};
void MotorControl(void *param)
{
	// 夹爪PID配置
	claw_motor.pos_pid.Kp = 0.6f;
	claw_motor.pos_pid.Ki = 0.0f;
	claw_motor.pos_pid.Kd = 0.3f;
	claw_motor.pos_pid.limit = 5.0f;
	claw_motor.pos_pid.output_limit = 10000.0f;

	claw_motor.vel_pid.Kp = 8.0f;
	claw_motor.vel_pid.Ki = 0.3f;
	claw_motor.vel_pid.Kd = 0.0f;
	claw_motor.vel_pid.limit = 2000.0f;
	claw_motor.vel_pid.output_limit = 10000.0f;

	// 推射电机 PID配置
	push_motor_pid.Kp = 1.2f;
	push_motor_pid.Ki = 0.0f;
	push_motor_pid.Kd = 0.3f;
	push_motor_pid.limit = 5.0f;
	push_motor_pid.output_limit = 120.0f;

	pitch_motor.vel_pid.Kp = 2.1f;
	pitch_motor.vel_pid.Ki = 0.3f;
	pitch_motor.vel_pid.Kd = 0.0f;
	pitch_motor.vel_pid.limit = 2000.0f;
	pitch_motor.vel_pid.output_limit = 16384.0f;
	
	pitch_motor.pos_pid.Kp=2.3f;
	pitch_motor.pos_pid.Ki=0.0f;
	pitch_motor.pos_pid.Kd=2.0f;
	pitch_motor.vel_pid.limit = 20.0f;
	pitch_motor.pos_pid.output_limit=16384.0f;

	//等待电机上电成功
	//while(!(claw_motor.ready&&push_motor.ready&&pitch_motor.ready&&jump_motor.ready))
	//	vTaskDelay(pdMS_TO_TICKS(100));
	
	while (1)
	{
		//跳跃/发射蓄力电机PID速度环控制
		//PID_Control2(jump_motor.motor.Speed, jump_motor_target_vel, &jump_motor.vel_pid);
		//运球/发射状态切换电机位置环控制
		PID_Control2(pitch_motor.motor.Angle-pitch_motor.offset, pitch_motor_target_pos, &pitch_motor.pos_pid);
		PID_Control2(pitch_motor.motor.Speed, pitch_motor.pos_pid.pid_out, &pitch_motor.vel_pid);
		// 运球机构PID闭环控制
		PID_Control2(claw_motor.motor.Angle-claw_motor.offset, claw_motor_target_pos, &claw_motor.pos_pid);
		PID_Control2(claw_motor.motor.Speed, claw_motor.pos_pid.pid_out, &claw_motor.vel_pid);
		if(push_motor_pos_mode)
		{
			PID_Control2(push_motor.posVelEstimateGet.position, push_motor_target_pos, &push_motor_pid);
			ODriveSetVelocity(&push_motor, push_motor_pid.pid_out,0.0f);
		}
		else
		{
			ODriveSetVelocity(&push_motor, push_motor_target_vel, 0.0f);
		}
		ODriveGetEncoderEstimate(&push_motor);
		
		//canSendBuf[2]=jump_motor.vel_pid.pid_out;
		//canSendBuf[0] = pitch_motor.vel_pid.pid_out;
		canSendBuf2[0] = claw_motor.vel_pid.pid_out;
		//canSendBuf2[1]=push_motor.vel_pid.pid_out;
		//MotorSend(&hcan1, 0x200, canSendBuf);
		MotorSend(&hcan2, 0x200, canSendBuf2);
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

//机构动作执行
void ActionDealTask(void* queue)
{
	Action_t action;
	while(1)
	{
		xQueueReceive((QueueHandle_t)queue,&action,portMAX_DELAY);
		action.action_cb(action.param);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint16_t ID = CAN_Receive_DataFrame(hcan, buf);
	Motor2006Recv(&claw_motor, hcan, ID, buf);
	Motor3508Recv(&pitch_motor, hcan, ID, buf);
	ODriveRecvServe(&push_motor, ID, buf);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint16_t ID = CAN_Receive_DataFrame(hcan, buf);
	Motor2006Recv(&claw_motor, hcan, ID, buf);
	Motor3508Recv(&pitch_motor, hcan, ID, buf);
	ODriveRecvServe(&push_motor, ID, buf);
}
