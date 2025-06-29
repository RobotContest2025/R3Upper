#include "motorControl.h"
#include "PID_old.h"
#include "Odrive.h"
#include "RobStride2.h"
#include "can.h"
#include "usart.h"

extern QueueHandle_t action_queue;
extern QueueHandle_t copilot_action_queue;
extern QueueHandle_t action_semaphore;

//Motor2006Ex_t motor1={.hcan=&hcan1,.ID=0x201};
//Motor2006Ex_t motor2={.hcan=&hcan1,.ID=0x202};
Motor2006Ex_t claw_motor = {.hcan = &hcan2, .ID = 0x203};
ODrive jump_motor1 = {.hcan = &hcan1, .motorID = 0x020};
ODrive jump_motor2 = {.hcan = &hcan1, .motorID = 0x010};
PID2 jump_motor_pid={.Kp=0.f,.Ki=0.005f,.limit=50.0f,.output_limit=50.0f};
RobStride_t throw_motor={.hcan=&hcan2,.motor_id=0x03,.host_id=0xFD,.type=RobStride_04};
PID2 throw_vel_pid,throw_pos_pid;

//������Ʊ���
uint8_t throw_motor_vel_mode=0;
uint8_t jump_motor_vel_mode=1;
uint8_t jump_motor_enable=1;
int32_t claw_motor_target_pos = 0;
float throw_motor_target_vel=0;
float throw_motor_target_torque=0;
float jump_motor_target_vel=0.0f;
float jump_motor_target_cur=0.0f;
float lift_motor1_target_vel=0.0f;
float lift_motor2_target_vel=0.0f;
float throw_motor_target_rad=0.0f;

uint8_t last_jump_motor_state=0;
// ���PID�ջ���������
int16_t canSendBuf[4] = {0};
int16_t canSendBuf2[4] = {0};

float debug_vel=0.0f;
float debug_pos=0.0f;
float debug_vol=0.0f;

int16_t max_2006_cur=3000;
float last_exp_vel=0.0f;
uint32_t cnt=0;

float motor_cur1,motor_cur2;

void MotorControl(void *param)
{
	// ��צPID����
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

	//motor1.vel_pid=claw_motor.vel_pid;	//����PID����
	//motor1.vel_pid.limit=12000.0f;	//���Ļ����޷�
	
	//������PID����
	throw_vel_pid.Kp=1.3f;
	throw_vel_pid.Ki=0.2f;
	throw_vel_pid.Kd=0.0f;
	throw_vel_pid.limit=50.0f;
	throw_vel_pid.output_limit=40.0f;
	
	throw_pos_pid.output_limit=5.0f;
	throw_pos_pid.Kp=8.0f;
	throw_pos_pid.Kd=200.0f;
	
	vTaskDelay(1000);
	//RobStride�����ʼ����ʼ
	RobStrideResetAngle(&throw_motor);
	vTaskDelay(50);
	RobStrideSetMode(&throw_motor,RobStride_Torque);
	RobStrideEnable(&throw_motor);
	vTaskDelay(50);
	//RobStride�����ʼ������

	//�ȴ�Odrive�ϵ�ɹ�
//	while(jump_motor1.heartBeatGet.axisState!=0x08)
		vTaskDelay(pdMS_TO_TICKS(50));
	
	//ODrive�����ʼ����ʼ
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
	//ODrive�����ʼ������
	
	
	//uint8_t buf1[8]={0x0b,0x20,0x00,0x00,0x78,0x00,0x00,0x00};
	//RobStrideSend(&throw_motor, (18<<24)|(throw_motor.host_id<<8)|(throw_motor.motor_id),buf1 );
	
	TickType_t last_wake_time = xTaskGetTickCount();
	while (1)
	{
		//RobStride/Odrive�����������
		RobStrideGet(&throw_motor,PARAM_MECH_POS);
		RobStrideGet(&throw_motor,PARAM_MECH_VEL);
		//ODriveGetEncoderEstimate(&jump_motor1);
		ODriveGetEncoderEstimate(&jump_motor2);
		
		jump_motor1.posVelEstimateGet.position=-jump_motor2.posVelEstimateGet.position;
		jump_motor1.posVelEstimateGet.velocity=-jump_motor2.posVelEstimateGet.velocity;
		
		vTaskDelay(2);

		//Ͷ�������ƻ�·
		if(throw_motor_vel_mode==2)
		{
			PID_Control2(throw_motor.state.rad, throw_motor_target_rad, &throw_pos_pid);
			throw_motor_target_vel=throw_pos_pid.pid_out;
		}
		if(throw_motor_vel_mode)
			PID_Control2(throw_motor.state.omega, throw_motor_target_vel, &throw_vel_pid);
		else
			throw_vel_pid.pid_out=0.0f;
		RobStrideTorqueControl(&throw_motor,throw_vel_pid.pid_out+throw_motor_target_torque);
		
		//��צ�����·
		PID_Control2(claw_motor.actual_pos, claw_motor_target_pos, &claw_motor.pos_pid);
		PID_Control2(claw_motor.motor.Speed, claw_motor.pos_pid.pid_out, &claw_motor.vel_pid);
		canSendBuf2[2]=claw_motor.vel_pid.pid_out;

        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET && claw_motor_target_pos == 0)
            limit(canSendBuf2[2], max_2006_cur, -max_2006_cur);

		MotorSend(&hcan2, 0x200, canSendBuf2);
		
		//��Ծ������ƻ�·
		if(jump_motor_vel_mode)	//�����ٶ�ģʽ�Ļ�����PID���������0
			PID_Control2(jump_motor1.posVelEstimateGet.velocity,jump_motor_target_vel,&jump_motor_pid);
		else
			jump_motor_pid.pid_out=0.0f;
		
		if(jump_motor_enable!=last_jump_motor_state)	//��Ծ�����Ҫ�л�ģʽ
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
			motor_cur1=jump_motor_pid.pid_out+jump_motor_target_cur;
			motor_cur2=-motor_cur1;
			ODriveSetTorque(&jump_motor1,motor_cur1);
			ODriveSetTorque(&jump_motor2,motor_cur2);	//�������
		}
		
		
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
//�����������񴴽�����
void ActionDealTask(void* queue)
{
	Action_t action={0},last_action={0};
	while(1)
	{
		xQueueReceive((QueueHandle_t)queue,&action,portMAX_DELAY);	//����һ���µĶ���ִ�е�����
		
		if(task_handle)		//������һ����������ִ��
		{
			if(last_action.type==ACTION_TYPE_INTERRUPTABLE)	//��һ�������ǿɴ�ϵģ�ֱ��ǿ�ƽ���
				vTaskDelete(task_handle);
			else	//��ǰ�����ǲ����жϵģ��ȴ��������(�����ڽ����������ź�������ɾ)
				xSemaphoreTake(action_semaphore,portMAX_DELAY);
		}
		
		if(choose_stack)
			task_handle=xTaskCreateStatic(action.action_cb,"action1",128,action.param,2,action_stack1,&task_block1);	//ִ�ж������������ȼ�Ϊ2��ȷ���������������ȼ�Ϊ3������ɾ��
		else
			task_handle=xTaskCreateStatic(action.action_cb,"action2",128,action.param,2,action_stack2,&task_block2);
		choose_stack=!choose_stack;		//�л���һ��ִ�ж���ʹ�õ�ջ�ռ�
		last_action=action;
	}
}

//��������������ִ������
void CoplitActionTask(void* queue)
{
	Action_t action;
	while(1)
	{
		xQueueReceive((QueueHandle_t)queue,&action,portMAX_DELAY);	//����һ���µĶ���ִ�е�����
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
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
	Motor2006Recv(&claw_motor, hcan, ID, buf);
	//ODriveRecvServe(&jump_motor1, ID, buf);
	//ODriveRecvServe(&jump_motor2, ID, buf);
	RobStrideRecv_Handle(&throw_motor, hcan, ID, buf);
}

uint32_t can1_err_cnt=0;
uint32_t can2_err_cnt=0;
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
		can1_err_cnt++;
	if(hcan->Instance==CAN2)
		can2_err_cnt++;
}
