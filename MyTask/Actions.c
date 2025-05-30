#include "Action.h"
#include "Odrive.h"
#include "RobStride2.h"


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
extern RobStride_t pitch_motor;
extern float pitch_motor_actual_pos;

extern uint8_t jump_motor_vel_mode;
extern uint8_t pitch_motor_pos_mode;
extern uint8_t jump_motor_enable;
extern uint8_t push_motor_vel_mode;
extern int32_t claw_motor_target_pos;
extern float push_motor_target_vel;
extern float push_motor_target_cur;
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
	jump_motor_enable=1;
	ActionFinished();
}



//extern float launch_exp_vel;
extern float launch_exp_dis;
void TestAction(void *param)
{
	UNUSED(param);
	//pitch_motor_target_cur=pitch_lock_cur;
	push_motor_vel_mode=0;
	
	float x=launch_exp_dis;
	test_launch_cur = 0.0002f*x*x*x + 0.0192*x*x + 9.4400*x + 345.7935f;

	
	push_motor_target_cur=test_launch_cur;
	while(push_motor.actual_pos<launch_gate_pos)
	{
		launch_pos=push_motor.actual_pos;
		push_motor_target_vel=push_motor.motor.Speed;
		vTaskDelay(5);
	}
	
	push_motor_vel_mode=1;
	MotorTargetTrack_float(0.0f, &push_motor_target_vel, 200.0f);
	//pitch_motor_target_cur=0.0f;	//释放俯仰电机
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;
	
	ActionFinished();
}

#if 0
float launch_vel=60000.0f;

void LaunchAction_coplit(void *param)
{
	UNUSED(param);
	pitch_motor_target_cur=pitch_lock_cur;
	push_motor_target_vel=launch_vel;
	while(push_motor.actual_pos<launch_gate_pos)
		vTaskDelay(5);
	MotorTargetTrack_float(0.0f, &push_motor_target_vel,200.0f);
	pitch_motor_target_cur=0.0f;
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;
}

float fast_jump_stop_pos=7.0f;
float fast_jump_launch_pos=4.9f;
float fast_jump_cur=25.0f;

void FastJump(void *param)
{
	Action_t action={.action_cb=LaunchAction_coplit};
	UNUSED(param);
	float reset_offset=0.0f;
	float vel_record[16]={1.0f};
	int record_point=0;
	float sum=0.0f;
	
	jump_motor_target_vel=-3.0f;
	jump_motor_vel_mode=1;
	jump_motor_enable=1;
	vTaskDelay(1000);
	do
	{
		vTaskDelay(50);
		vel_record[record_point]=jump_motor1.posVelEstimateGet.velocity;
		record_point=(record_point+1)%16;
		sum=0.0f;
		for(int i=0;i<16;i++)
			sum+=ABS(vel_record[i]);
		sum=sum/16.0f;
	}while(sum>0.01f);	//速度滤波
	reset_offset=jump_motor1.posVelEstimateGet.position;		//到达最低端，并记录最低端的位置
	
	jump_motor_target_cur=fast_jump_cur;
	jump_motor_vel_mode=0;
	jump_motor_target_vel=0.0f;
	while(jump_motor1.posVelEstimateGet.position<fast_jump_launch_pos+reset_offset)//等待直到加速到期望速度或者到达刹车点
		vTaskDelay(5);
	
	xQueueSend(copilot_action_queue,&action,0);		//执行发射动作
	
	while(jump_motor1.posVelEstimateGet.position<fast_jump_stop_pos+reset_offset)//等待直到加速到期望速度或者到达刹车点
		vTaskDelay(5);
	
	jump_motor_target_cur=0.0f;
	jump_motor_enable=0;
	do{
		vTaskDelay(5);
	}while(jump_motor1.posVelEstimateGet.velocity>-5.0f);		//在撞击时关闭电机，防止控制器出错
	jump_motor_enable=1;

	jump_motor_target_cur=0.0f;
	jump_motor_target_vel=-3.0f;
	jump_motor_vel_mode=1;
	while(jump_motor1.posVelEstimateGet.position>fast_jump_stop_pos+reset_offset+1.0f)	//等待直到到达刹车点
		vTaskDelay(5);
	jump_motor_target_vel=0.0f;
	
	ActionFinished();
}

float base_launch_height=1.355f;	//参考发射状态下，发射器的高度(m)

float launch_exp_vel=3.42295f;
float launch_exp_dis=2.47f;
void LaunchInVel(void* param)	//以某个特定的速度发射
{
	UNUSED(param);
	launch_exp_vel=launch_exp_dis/sqrtf(2.0f*base_launch_height/9.802f);
	push_motor_vel_mode=0;				//启用速度PID闭环
	push_motor_target_cur=GetLaunchTorqueEstimate(launch_exp_vel);
	push_motor_target_vel=GetCurrentExpVelocity(push_motor.actual_pos,launch_exp_vel);
	while(push_motor.actual_pos<launch_gate_pos)	//进行速度前馈控制直到到达末端
	{
		push_motor_target_vel=GetCurrentExpVelocity(push_motor.actual_pos,launch_exp_vel);
		vTaskDelay(2);
	}
	push_motor_target_cur=0.0f;
	push_motor_vel_mode=1;
	MotorTargetTrack_float(0.0f, &push_motor_target_vel,200.0f);	//执行刹车过程
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;		//复位推射机构
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;

	ActionFinished();
}

//vx=6,vy=7,dt=0.4
float exp_vx=5.0f;//4.85f;
float exp_vy=6.0f;//=5.30f;
float k_x_r=8.0f/0.86f;     //距离*k=电机圈数
float k_a_i=12.4263464f;           //加速度*k=电机力矩电流
float mass_compensate=0.5f;    //质量补偿
float jump_start_cur=30.0f;
float jump_stop_pos_r=7.7f;
float reset_offset=0.0f;


float remain_time;
float launch_consume=0.7f;

void JumpInVel(void* param)
{
	Action_t action={.action_cb=LaunchInVel_coplit};
	UNUSED(param);
	float vel_record[16]={1.0f};
	int record_point=0;
	float sum=0.0f;
	
  jump_motor_target_cur=0.0f;
	jump_motor_target_vel=-3.0f;
	jump_motor_vel_mode=1;
	jump_motor_enable=1;
	while(jump_motor1.posVelEstimateGet.position>reset_offset+0.1f)
		vTaskDelay(5);


	//launch_consume=0.3f;//GetLaunchTimeEstimate(exp_vx); //水平投射机构将装置投出去时所需要的电流
    //float a=exp_vy*exp_vy/(2*jump_stop_pos_r/k_x_r);    //计算起跳时的加速度
	//jump_motor_target_cur=a/k_a_i;	                    //给一个合适的起跳电流作为力矩前馈
    jump_motor_target_cur=0.0f;//jump_start_cur;               //加速段给最大加速电流
    jump_motor_target_vel=exp_vy*k_x_r;
	jump_motor_vel_mode=1;
	jump_motor_enable=1;
	
    //remain_time=sqrtf((2.0f*jump_stop_pos_r/k_x_r)/a);
    //float k_filter=0.3f;
    //float last_vel=0.0f;

    //while(jump_motor1.posVelEstimateGet.position<+reset_offset+1.0f)//等待恒力弹簧收紧
		//	vTaskDelay(2);
	
		//jump_motor_target_vel=jump_motor1.posVelEstimateGet.velocity;
		//jump_motor_vel_mode=1;
		
	
    /*do{
        float motor_vel=k_filter*jump_motor1.posVelEstimateGet.velocity/k_x_r+(1-k_filter)*last_vel;                                //对采集到的电机速度进行低通滤波
        float motor_pos=((jump_motor1.posVelEstimateGet.position-reset_offset))/k_x_r;
        jump_motor_target_vel=sqrtf(2.0f*a*(jump_stop_pos_r/k_x_r-motor_pos)/k_x_r+motor_vel*motor_vel);
        remain_time=(-motor_vel+sqrtf(motor_vel*motor_vel+2*a*motor_pos))/a;   //计算跳跃过程的剩余时间
        a=(exp_vy*exp_vy-motor_vel*motor_vel)/(2*jump_stop_pos_r/k_x_r-motor_pos);
				jump_motor_target_cur=a/k_a_i;
        last_vel=motor_vel;
        vTaskDelay(5);
    }while(remain_time>launch_consume);	//还未到达启动发射进程的位置*/

    while(jump_stop_pos_r-(jump_motor1.posVelEstimateGet.position-reset_offset)>launch_consume*exp_vy)
    {
        /*if(jump_motor1.posVelEstimateGet.velocity-exp_vy*k_x_r>5.0f)
            jump_motor_target_cur=jump_start_cur;
        else
            jump_motor_target_cur=0.0f;*/
        vTaskDelay(3);
    }

	xQueueSend(copilot_action_queue,&action,0);		//执行发射动作
	
		//jump_motor_target_vel=exp_vy*k_x_r;
		//jump_motor_target_cur=0.0f;
	while(jump_motor1.posVelEstimateGet.position-reset_offset<jump_stop_pos_r)//等待直到到达刹车点
	{
        /*if(jump_motor1.posVelEstimateGet.velocity-exp_vy*k_x_r>5.0f)
            jump_motor_target_cur=jump_start_cur;
        else
            jump_motor_target_cur=0.0f;*/
        /*float motor_vel=k_filter*jump_motor1.posVelEstimateGet.velocity/k_x_r+(1-k_filter)*last_vel;                                 //对采集到的电机速度进行低通滤波
        float motor_pos=((jump_motor1.posVelEstimateGet.position-reset_offset))/k_x_r;
        jump_motor_target_vel=sqrtf(2.0f*a*(jump_stop_pos_r/k_x_r-motor_pos)/k_x_r+motor_vel*motor_vel);*/
        vTaskDelay(3);
  }

	jump_motor_target_cur=0.0f;
	jump_motor_target_vel=0.0f;
	jump_motor_enable=0;        //准备撞击桅杆顶端，失能电机

	do{
		vTaskDelay(5);
	}while(jump_motor1.posVelEstimateGet.velocity>0.0f);		//在撞击时关闭电机，防止控制器出错
	jump_motor_enable=1;
	
	jump_motor_target_cur=0.0f;
	jump_motor_target_vel=-3.0f;
	jump_motor_vel_mode=1;
	while(jump_motor1.posVelEstimateGet.position>reset_offset+3.0f)	//限速下降到复位点
		vTaskDelay(5);
	jump_motor_target_vel=0.0f;

	ActionFinished();
}

void LaunchInVel_coplit(void* param)	//以某个特定的速度发射(使用副队列)
{
	UNUSED(param);
	push_motor_vel_mode=0;
	push_motor_target_cur=GetLaunchTorqueEstimate(exp_vx);
	//push_motor_target_vel=GetCurrentExpVelocity(exp_vx);
	while(push_motor.actual_pos<launch_gate_pos)	//进行速度前馈控制直到到达末端
	{
		push_motor_target_vel=GetCurrentExpVelocity(push_motor.actual_pos,exp_vx);
		vTaskDelay(2);
	}
	push_motor_target_cur=0.0f;
	push_motor_vel_mode=1;
	MotorTargetTrack_float(0.0f, &push_motor_target_vel,200.0f);	//执行刹车过程
	vTaskDelay(1000);
	push_motor_target_vel=-500.0f;		//复位推射机构
	while(push_motor.actual_pos>0.0f)
		vTaskDelay(5);
	push_motor_target_vel=0.0f;
}

float base_launch_dis=2.42f;	//参考发射状态下，发射的距离(m)
//float base_launch_height=1.3f;	//参考发射状态下，发射器的高度(m)
//float base_launch_vel=4.69705f;//4.79587249622f;	//参考发射状态下，发射的速度(m/s)
float base_launch_vel=4.69879f;
float base_launch_torque=8000.0f;	//参考发射状态下，电机的Iq值（非标）
float base_launch_time=0.289f;	//参考发射状态下，发射全过程所需要的时间(s)

//自由落体时间0.525861f
int debug_dt;
float debug_jump_cur=5.0f;
void DebugAction(void* param)
{
    debug_dt=HAL_GetTick();
    jump_motor_target_cur=debug_jump_cur;
		jump_motor_vel_mode=0;
    while(jump_motor1.posVelEstimateGet.position<7.0f+reset_offset)//等待直到到达刹车点
        vTaskDelay(5);
		
		jump_motor_target_cur=0;
		jump_motor_enable=0;        //准备撞击桅杆顶端，失能电机
	do{
		vTaskDelay(5);
	}while(jump_motor1.posVelEstimateGet.velocity>0.0f);		//在撞击时关闭电机，防止控制器出错
	jump_motor_enable=1;
		
	jump_motor_target_cur=0.0f;
	jump_motor_target_vel=-3.0f;
	jump_motor_vel_mode=1;
	while(jump_motor1.posVelEstimateGet.position>reset_offset+3.0f)	//限速下降到复位点
		vTaskDelay(5);
	jump_motor_target_vel=0.0f;
	
    jump_motor_target_cur=0.0f;
    debug_dt=HAL_GetTick()-debug_dt;
    jump_motor_enable=0;
    ActionFinished();
}

const float interp_table[][2] = {
     {2, -282},
    {12, -275},
    {41, -259},
    {41, -232},
    {87, -190},
    {156, -190},
    {255, -128},
    {366, -50},
    {470, 32},
    {470, 126},
    {565, 126},
    {675, 248},
    {730, 351},
    {784, 469},
    {883, 612},
    {883, 766},
    {933, 938},
    {1060, 938},
    {1150, 1104},
    {1207, 1280},
    {1207, 1503},
    {1347, 1702},
    {1379, 1702},
    {1487, 1925},
    {1623, 2168},
    {1646, 2409},
    {1799, 2688},
    {1799, 2947},
    {1864, 3243},
    {1941, 3243},
    {2034, 3530},
    {2111, 3852},
    {2211, 4158},
    {2211, 4507},
    {2308, 4831},
    {2375, 4831},
    {2469, 5205},
    {2571, 5578},
    {2626, 5948},
    {2626, 6347},
    {2762, 6767},
    {2848, 6767},
    {2919, 7176},
    {3000, 7597},
    {3103, 8044},
    {3103, 8517},
    {3182, 8964},
    {3264, 8964},
    {3369, 9438},
    {3436, 9933},
    {3528, 10441},
    {3528, 10960},
    {3651, 11488},
    {3731, 11488},
    {3810, 12028},
    {3873, 12576},
    {3959, 13138},
    {3959, 13716},
    {4063, 14348},
    {4164, 14348},
    {4285, 14958},
    {4369, 15570},
    {4444, 16151},
    {4444, 16833},
    {4555, 17476},
    {4610, 18133},
    {4689, 18133},
    {4790, 18808},
    {4844, 19491},
    {4844, 20194},
    {4938, 20911},
    {5047, 20911},
    {5147, 21632},
    {5231, 22358},
    {5338, 23118},
    {5338, 23866},
    {5421, 24632},
    {5474, 24632},
    {5574, 25415},
    {5645, 26207},
    {5733, 27009},
    {5733, 27827},
    {5825, 28655},
    {5922, 29489},
    {6007, 29489},
    {6083, 30344},
    {6181, 31267},
    {6181, 32079},
    {6259, 32964},
    {6328, 32964},
    {6424, 33861},
    {6491, 34761},
    {6579, 35682},
    {6579, 36611},
    {6660, 37554},
    {6748, 38505},
    {6836, 38505},
    {6907, 39474},
    {7004, 40435},
    {7073, 41432},
    {7073, 42415},
    {7148, 43423},
    {7232, 43423},
    {7311, 44440},
    {7372, 45471},
    {7474, 46505},
    {7474, 47557},
    {7541, 48609},
    {7627, 48609},
    {7719, 49686},
    {7759, 50758},
    {7838, 51849},
    {7838, 52940},
    {7903, 54050},
    {7981, 54050},
    {8069, 55166},
    {8142, 56297},
    {8215, 57435},
    {8215, 58585},
    {8288, 59742},
    {8366, 59742},
    {8422, 60905},
    {8491, 62083},
    {8539, 63257},
    {8539, 64445},
    {8606, 65638},
    {8652, 65638},
    {8707, 66835},
    {8747, 68040},
    {8776, 69248},
    {8776, 70467},
    {8837, 71687},
    {8860, 71687},
    {8904, 72992},
    {8941, 74144},
    {8948, 75182},
    {8948, 76593},
    {8983, 77898},
    {8989, 77898},
    {9002, 79053},
    {9015, 80364},
    {9008, 81587},
    {9008, 82824},
    {9017, 84054},
    {9004, 84054},
    {9008, 85284},
    {9012, 86520},
    {9010, 87747},
    {9010, 88977},
    {9035, 90202},
    {9015, 90202},
    {9010, 91426},
    {8989, 92652},
    {8956, 93872},
    {8956, 95096},
    {8966, 96314},
    {8935, 96314},
    {8918, 97529},
    {8891, 98737},
    {8841, 99926},
    {8841, 101119},
    {8805, 102302},
    {8753, 102302},
    {8709, 103480},
    {8652, 104644},
    {8586, 105798},
    {8508, 106944},
    {8508, 108064},
    {8416, 109184},
    {8337, 109184},
    {8242, 110284},
    {8169, 111378},
    {8075, 112452},
    {8075, 113514},
    {7979, 114562},
    {7847, 114562},
    {7732, 115594},
    {7621, 116603},
    {7510, 117609},
    {7510, 118600},
    {7420, 119568},
    {7319, 119568},
    {7206, 120524},
    {7087, 121478},
    {6993, 122390},
    {6993, 123325},
    {6874, 124216},
    {6790, 124216},
    {6694, 125121},
    {6604, 125993},
    {6522, 126874},
    {6522, 127732},
    {6453, 128592},
    {6376, 128592},
    {6315, 129437},
    {6242, 130278},
    {6171, 131095},
    {6171, 131922},
    {6118, 132739},
    {6053, 132739},
    {6022, 133545},
    {5949, 134351},
    {5917, 135145},
    {5917, 135930},
    {5873, 136721},
    {5800, 136721},
    {5788, 137497},
    {5719, 138270},
    {5677, 139045},
    {5677, 139814},
    {5698, 140571},
    {5639, 140571},
    {5629, 141397},
    {5631, 142154},
    {5578, 142865},
    {5587, 143678},
    {5587, 144444},
    {5585, 145196},
    {5576, 145196},
    {5570, 145967},
    {5564, 146720},
    {5564, 147466},
    {5564, 148218},
    {5537, 148218},
    {5503, 148968},
    {5486, 149713},
    {5468, 150444},
    {5468, 151178},
    {5421, 151923},
    {5411, 151923},
    {5396, 152660},
    {5384, 153389},
    {5398, 154138},
    {5403, 154877},
    {5403, 155608},
    {5413, 156354},
    {5428, 156354},
    {5430, 157101},
    {5453, 157863},
    {5497, 158613},
    {5497, 159373},
    {5505, 160154},
    {5555, 160154},
    {5616, 160929},
    {5637, 161707},
    {5714, 162509},
    {5714, 163296},
    {5777, 164115},
    {5800, 164115},
    {5865, 164927},
    {5911, 165743},
    {5966, 166586},
    {6058, 167420},
    {6058, 168279},
    {6116, 168279},
    {6194, 169145},
    {6267, 170018},
    {6359, 170900},
    {6359, 171801},
    {6413, 172708},
    {6518, 172708},
    {6579, 173627},
    {6652, 174553},
    {6742, 175492},
    {6742, 176436},
    {6790, 177389},
	{6882, 177389},
    {6903, 178344},
    {6957, 179296},
    {6962, 180297},
    {6945, 181182},
    {6945, 182092},
    {6905, 182092},
    {6803, 182998},
    {6692, 183866},
    {6533, 184780},
    {6533, 185550},
    {6380, 186455},
    {6198, 186455},
    {6030, 187133},
    {5865, 187878},
    {5660, 188674},
    {5660, 189345},
    {5522, 190041}
};

float linearInterpolation(int tab_size, float x, const float table[][2]) {
    if (x <= table[0][1])
        return table[0][0];
    if (x >= table[tab_size - 1][1])
        return table[tab_size - 1][0];

    for (int i = 0; i < tab_size - 1; ++i) {
        float x0 = table[i][1];
        float y0 = table[i][0];
        float x1 = table[i + 1][1];
        float y1 = table[i + 1][0];

        if (x >= x0 && x <= x1) {
            return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
        }
    }

    return 0.0f; // 理论上不会执行到这里
}

float GetLaunchTimeEstimate(float vx)	//计算以某个速度发射篮球所需要的时间
{
	float k=base_launch_time*base_launch_vel;
	
	if(vx<1.0f)		//防止除0
		return 2.0f;
	
	return k/vx;
}

float debug_value3=0.0f;

float GetLaunchTorqueEstimate(float vx)				//计算以某个速度发射时需要的力矩
{
	float k=vx/base_launch_vel;
	return base_launch_torque*k*k;
}

float GetCurrentExpVelocity(float fai,float exp_v)
{
	float k=exp_v/base_launch_vel;
	float base_current_motor_vel=linearInterpolation(sizeof(interp_table)/8,fai,interp_table);//TODO:打个表得到一个电机机械角度和转子速度的关系式
	return k*base_current_motor_vel;	//以特定的比例缩放得到电机在标准发射状态下的转速
}
#endif 



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
