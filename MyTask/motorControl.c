#include "motorControl.h"
#include "PID_old.h"
#include "Odrive.h"
#include "RobStride2.h"
#include "can.h"
#include "usart.h"

extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;
extern QueueHandle_t action_semaphore;

Motor2006Ex_t motor1={.hcan=&hcan1,.ID=0x201};
Motor2006Ex_t motor2={.hcan=&hcan1,.ID=0x202};
Motor2006Ex_t claw_motor = {.hcan = &hcan1, .ID = 0x201};
ODrive jump_motor1 = {.hcan = &hcan2, .motorID = 0x010};
ODrive jump_motor2 = {.hcan = &hcan2, .motorID = 0x020};
PID2 jump_motor_pid={.Kp=0.1f,.Ki=0.005f,.limit=50.0f,.output_limit=5.0f};
RobStride_t throw_motor={.hcan=&hcan2,.motor_id=0x03,.host_id=0xFD,.type=RobStride_04};
PID2 throw_vel_pid;

//电机控制变量
uint8_t throw_motor_vel_mode=0;
uint8_t jump_motor_vel_mode=0;
uint8_t jump_motor_enable=1;
int32_t claw_motor_target_pos = 0;
float throw_motor_target_vel=0;
float throw_motor_target_iq=0;
float jump_motor_target_vel=0.0f;
float jump_motor_target_cur=0.0f;
float lift_motor1_target_vel=0.0f;
float lift_motor2_target_vel=0.0f;

uint8_t last_jump_motor_state=0;
// 电机PID闭环控制任务
int16_t canSendBuf[4] = {0};
int16_t canSendBuf2[4] = {0};

float debug_vel=0.0f;
float debug_pos=0.0f;
float debug_vol=0.0f;
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
	claw_motor.vel_pid.limit = 100.0f;
	claw_motor.vel_pid.output_limit = 10000.0f;

	motor1.vel_pid=claw_motor.vel_pid;	//复制PID参数
	motor1.vel_pid.limit=12000.0f;	//更改积分限幅
	
	//抛射电机PID配置
	throw_vel_pid.Kp=1.3f;
	throw_vel_pid.Ki=0.2f;
	throw_vel_pid.Kd=0.0f;
	throw_vel_pid.limit=50.0f;
	throw_vel_pid.output_limit=40.0f;


	//等待电机上电成功
	//while(!(claw_motor.ready&&push_motor.ready&&throw_motor.ready&&jump_motor.ready))
	//	vTaskDelay(pdMS_TO_TICKS(100));
	
	//RobStride电机初始化开始
	RobStrideResetAngle(&throw_motor);
	vTaskDelay(50);
	RobStrideSetMode(&throw_motor,RobStride_Torque);
	RobStrideEnable(&throw_motor);
	vTaskDelay(50);
	//RobStride电机初始化结束
	
	//ODrive电机初始化开始
	int axis_state=ODRIVE_SET_AXIS_STATE_IDLE;
	ODriveSetAxisState(&jump_motor1,axis_state);
	ODriveSetAxisState(&jump_motor2,axis_state);
	vTaskDelay(1);
	ODriveSetControlMode(&jump_motor1,ODRIVE_SET_WOORKMODE_TORQUE_CONTROL|ODRIVE_SET_INPUTMODE_PASSTHROUGH);
	ODriveSetControlMode(&jump_motor2,ODRIVE_SET_WOORKMODE_TORQUE_CONTROL|ODRIVE_SET_INPUTMODE_PASSTHROUGH);
	vTaskDelay(1);
	axis_state=ODRIVE_SET_AXIS_STATE_CLOSED_LOOP_CONTROL;
	ODriveSetAxisState(&jump_motor1,axis_state);
	ODriveSetAxisState(&jump_motor2,axis_state);
	vTaskDelay(1);
	//ODrive电机初始化结束
	
	TickType_t last_wake_time = xTaskGetTickCount();
	while (1)
	{
		//RobStride/Odrive电机数据请求
		RobStrideGet(&throw_motor,PARAM_MECH_POS);
		RobStrideGet(&throw_motor,PARAM_MECH_VEL);
		ODriveGetEncoderEstimate(&jump_motor1);
		vTaskDelay(2);

		//投射电机控制环路
		if(throw_motor_vel_mode)
			PID_Control2(throw_motor.state.omega, throw_motor_target_vel, &throw_vel_pid);
		else
			throw_vel_pid.pid_out=0.0f;
		RobStrideTorqueControl(&throw_motor,throw_vel_pid.pid_out+throw_motor_target_iq);
		
		//夹爪电机环路
		//PID_Control2(claw_motor.actual_pos, claw_motor_target_pos, &claw_motor.pos_pid);
		//PID_Control2(claw_motor.motor.Speed, claw_motor.pos_pid.pid_out, &claw_motor.vel_pid);
		//canSendBuf2[0]=claw_motor.vel_pid.pid_out;
		//MotorSend(&hcan2, 0x200, canSendBuf2);

		//主桅杆抬升电机环路
		PID_Control2(motor1.motor.Speed, lift_motor1_target_vel, &motor1.vel_pid);
		PID_Control2(motor2.motor.Speed, lift_motor2_target_vel, &motor2.vel_pid);
		canSendBuf2[0]=(int16_t)motor1.vel_pid.pid_out;
		canSendBuf2[1]=(int16_t)motor2.vel_pid.pid_out;
		MotorSend(&hcan1, 0x200, canSendBuf2);
		
		//跳跃电机控制环路
		if(jump_motor_vel_mode)	//不是速度模式的话，将PID输出变量归0
			PID_Control2(jump_motor1.posVelEstimateGet.velocity,jump_motor_target_vel,&jump_motor_pid);
		else
			jump_motor_pid.pid_out=0.0f;
		
		if(jump_motor_enable!=last_jump_motor_state)	//跳跃电机需要切换模式
		{
			last_jump_motor_state=jump_motor_enable;
			if(jump_motor_enable)
			{
				int axis_state=ODRIVE_SET_AXIS_STATE_CLOSED_LOOP_CONTROL;
				ODriveSetAxisState(&jump_motor1,axis_state);
				ODriveSetAxisState(&jump_motor2,axis_state);
			}
			else
			{
				int axis_state=ODRIVE_SET_AXIS_STATE_IDLE;
				ODriveSetAxisState(&jump_motor1,axis_state);
				ODriveSetAxisState(&jump_motor2,axis_state);
			}
			vTaskDelay(1);
		}
		else
		{
			float motor_cur=jump_motor_pid.pid_out+jump_motor_target_cur;
			ODriveSetTorque(&jump_motor1,motor_cur);
			motor_cur=-motor_cur;
			ODriveSetTorque(&jump_motor2,motor_cur);	//电机控制
		}
		//vTaskDelay(1);
		//ODriveSendOrReceiveData(&jump_motor1,ODRIVE_CMD_READ|ODRIVE_GET_TORQUE_CURRENT,NULL);
		//ODriveGetPowerInfo(&jump_motor1);
		//debug_vel=jump_motor1.posVelEstimateGet.velocity;
		//debug_vol=jump_motor1.powerGet.Voltage;
		
		vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(5));
	}
}

static uint32_t action_stack1[128];
static uint32_t action_stack2[128];
static StaticTask_t task_block1;
static StaticTask_t task_block2;
static uint8_t choose_stack=0;
static TaskHandle_t task_handle=NULL;
uint32_t task_exit=0;
//机构动作任务创建任务
void ActionDealTask(void* queue)
{
	Action_t action;
	while(1)
	{
		if(task_handle)		//可能有一个任务正在执行
		{
			if(action.type==ACTION_TYPE_INTERRUPTABLE)	//当前任务是可中断的，直接强制结束
				vTaskDelete(task_handle);
			else	//当前任务是不可中断的，等待任务结束(任务在结束后会给出信号量并自删)
				xSemaphoreTake(action_semaphore,portMAX_DELAY);
		}
		
		xQueueReceive((QueueHandle_t)queue,&action,portMAX_DELAY);	//接收一个新的动作执行的请求
		if(choose_stack)
			task_handle=xTaskCreateStatic(action.action_cb,"action1",128,action.param,2,action_stack1,&task_block1);	//执行动作的任务优先级为2，确保可以立即被优先级为3的任务删除
		else
			task_handle=xTaskCreateStatic(action.action_cb,"action2",128,action.param,2,action_stack2,&task_block2);
		choose_stack=!choose_stack;		//切换下一次执行动作使用的栈空间
	}
}

//机构动作副动作执行任务
void CoplitActionTask(void* queue)
{
	Action_t action;
	while(1)
	{
		xQueueReceive((QueueHandle_t)queue,&action,portMAX_DELAY);	//接收一个新的动作执行的请求
		action.action_cb(action.param);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
	Motor2006Recv(&claw_motor, hcan, ID, buf);
	ODriveRecvServe(&jump_motor1, ID, buf);
	ODriveRecvServe(&jump_motor2, ID, buf);
	RobStrideRecv_Handle(&throw_motor, hcan, ID, buf);
	Motor2006Recv(&motor1, hcan, ID, buf);
	Motor2006Recv(&motor2, hcan, ID, buf);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
	Motor2006Recv(&claw_motor, hcan, ID, buf);
	ODriveRecvServe(&jump_motor1, ID, buf);
	ODriveRecvServe(&jump_motor2, ID, buf);
	RobStrideRecv_Handle(&throw_motor, hcan, ID, buf);
	Motor2006Recv(&motor1, hcan, ID, buf);
	Motor2006Recv(&motor2, hcan, ID, buf);
}
