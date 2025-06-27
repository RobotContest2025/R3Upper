#include "MoveControl.h"
#include "PID_old.h"
#include "arm_math.h"
#include "slope.h"
#include "usart.h"

extern float max_speed;                                      
extern float max_omega; 

// 需要在RemoteControl中分配按键控制这3个变量
uint8_t enable_offset = 0;  // 是否启用偏移自动校正

ChassisCtrl_t chassis = {.head = 0x5A}, remote;
RobotState_t robot;
PID2 pid2 = {.Kp = -4.0f, .Ki = -0.1f, .Kd = -2.0f, .limit = 100.0f, .output_limit = 72.0f};

uint8_t Attack_Defend_side = 1; //攻守方标志位 0为守方 1为攻方
Side_Data_Typedef Side_Data = {
    .Attack_X_BallHoop = 4000,      //攻方篮筐X坐标
    .Attack_Y_BallHoop = 975,       //攻方篮筐Y坐标
    .Defend_X_BallHoop = 4000,      //守方篮筐X坐标
    .Defend_Y_BallHoop = 0,         //守方篮筐Y坐标
};
velocity_Ex_Typedef Transformed_Ex;
/*云台跟随*/
uint8_t PTZ_followl = 0,                //云台跟随模式
        PID_Mode_Switch[2];             //云台跟随PID使能
float Robot_Launch = 0,                 //所使用机构方向和车头夹角 
      X_BallHoop = 0,                   //目标球框的坐标
      Y_BallHoop = 1000,                //目标球框的坐标
      PTZ_center[2],                    //发射机构在世界坐标系下的坐标
      E_PTZ_BallHoop = 0;               //世界坐标系下的发射机构与篮筐夹角
float Robot_Launch_Angle;               //发射机构发射方向与车头夹角
#define PTZ_center_Angle   0            //发射机构方向与车头夹角
#define PTZ_center_L 	   0            //发射机构与车中心的距离
FuzzyRule_t PTZ_followl_PID;
kalman_filter_t PTZ_Kalman[3];
uint8_t Last_PTZ_followl = 0;
#define PTZ_Kalman_Q 1
#define PTZ_Kalman_R 100000
velocity_Ex_Typedef KF_Pos;
/*云台跟随*/
/*绝对 圆周运动*/
uint8_t Circular_To_Absolute_motion = 0;    //绝对运动与圆周运动切换
float E_Body_BallHoop = 0,                  //车中心坐标与篮筐夹角
      Remote_Compensation_Factor = 0,       //防止车不在圆上的遥控补偿系数
      Remote_Compensate_velocity = 0,       //测量向心速度补偿
      V_Compensation_Factor = 0,            //防止车不在圆上的速度补偿系数
      V_Compensate_Velocity = 0,            //测量向心速度补偿
      Closed_Loop_Angle = 0;                //闭环角度
/*绝对 圆周运动*/
/*拦截姿态导航点*/
uint8_t Intercept_Flag = 0;     //1为使能
float Enemy_Hoop_Angle = 0,     //敌方和篮筐连线与场地的角度
      Navigate_Point[3],        //导航期望点坐标
      World_EV[2];              //世界坐标系下的期望速度
#define Safe_Distance 1000.f    //拦截安全距离
/*拦截姿态导航点*/


void MoveControlTask(void *param)
{
    Fuzzy_Rule_Init(13, 0.08, 11800, 40, 10, 1200, 0.2, &PTZ_followl_PID);

    for(uint8_t i = 0; i < 3; i++)
        kalman_Init(PTZ_Kalman + i, PTZ_Kalman_Q, PTZ_Kalman_R, 0);

    TickType_t last_wake_time = xTaskGetTickCount(); // 获取当前时间戳
    while (1)
    {
        memcpy(&chassis, &remote, sizeof(ChassisCtrl_t));
        if(PTZ_followl == 1 || Circular_To_Absolute_motion == 1){
            Transformed_Ex.Z = remote.omega;
            if(Attack_Defend_side == 1){//若为攻方*********************************************************************
                X_BallHoop = Side_Data.Attack_X_BallHoop;   Y_BallHoop = Side_Data.Attack_Y_BallHoop;   
                Robot_Launch = Robot_Launch_Angle;
            }else if(Attack_Defend_side == 0){//若为守方****************************************************************
                X_BallHoop = Side_Data.Defend_X_BallHoop;   Y_BallHoop = Side_Data.Defend_Y_BallHoop;   
                Robot_Launch = Robot_Launch_Angle + 180;//假设防守机构是进攻机构的反面
            }
            if(PTZ_followl == 1){
                if(Last_PTZ_followl != PTZ_followl){
                    kalman_Init(&PTZ_Kalman[0], PTZ_Kalman_Q, PTZ_Kalman_R, robot.x);
                    kalman_Init(&PTZ_Kalman[1], PTZ_Kalman_Q, PTZ_Kalman_R, robot.y);
                }
                KF_Pos.X = Kalman_Filter(&PTZ_Kalman[0], robot.x);
                KF_Pos.Y = Kalman_Filter(&PTZ_Kalman[1], robot.y);
                float sin_ang, cos_ang;  
                if(Attack_Defend_side == 1){//若为攻方*********************************************************************
                    E_Body_BallHoop = RAD2ANGLE(-atan2f(X_BallHoop - KF_Pos.X, Y_BallHoop - KF_Pos.Y));
                }else if(Attack_Defend_side == 0){//若为守方****************************************************************
                    E_Body_BallHoop = reform(RAD2ANGLE(-atan2f(X_BallHoop - KF_Pos.X, Y_BallHoop - KF_Pos.Y)) + 180);
                }
                
                arm_sin_cos_f32(E_Body_BallHoop, &sin_ang, &cos_ang);
                float Circular_Remote_Control[2];
                Circular_Remote_Control[0] = remote.v_x * cos_ang - remote.v_y * sin_ang;
                Circular_Remote_Control[1] = remote.v_x * sin_ang + remote.v_y * cos_ang;
                robot.Distance = sqrtf((KF_Pos.X-X_BallHoop)*(KF_Pos.X-X_BallHoop)+(KF_Pos.Y-Y_BallHoop)*(KF_Pos.Y-Y_BallHoop));
                Remote_Compensate_velocity = Circular_Remote_Control[0] * Circular_Remote_Control[0] / robot.Distance * Remote_Compensation_Factor;//计算期望向心速度补偿
                float line_vel = robot.v_x * cos_ang - robot.v_y * sin_ang; // 计算机器人在当前圆周上运动时的线速度
                V_Compensate_Velocity = line_vel * line_vel / robot.Distance * V_Compensation_Factor; //计算测量向心速度补偿
                Transformed_Ex.X = Circular_Remote_Control[0];
                if(Attack_Defend_side == 1)//若为攻方*********************************************************************
                    Transformed_Ex.Y = Circular_Remote_Control[1] + Remote_Compensate_velocity + V_Compensate_Velocity;
                else if(Attack_Defend_side == 0)//若为守方****************************************************************   
                    Transformed_Ex.Y = Circular_Remote_Control[1] - Remote_Compensate_velocity - V_Compensate_Velocity;

                arm_sin_cos_f32(reform(PTZ_center_Angle + KF_Pos.Z), &sin_ang, &cos_ang);
                PTZ_center[0] = KF_Pos.X - PTZ_center_L * sin_ang;
                PTZ_center[1] = KF_Pos.Y + PTZ_center_L * cos_ang;

                E_PTZ_BallHoop = -reform(RAD2ANGLE(atan2f(X_BallHoop - PTZ_center[0], Y_BallHoop - PTZ_center[1])) -Robot_Launch);
                if(fabsf(E_PTZ_BallHoop - robot.angle) > 180) E_PTZ_BallHoop -= 360*(((int)(E_PTZ_BallHoop - robot.angle)%360)/180 + (int)(E_PTZ_BallHoop - robot.angle)/360);//检测多圈旋转劣弧
                if(Last_PTZ_followl != PTZ_followl)
                    kalman_Init(&PTZ_Kalman[2], 10, 1000, E_PTZ_BallHoop);
                if(fabsf(robot.angle - E_PTZ_BallHoop) < 1.f)
                    KF_Pos.Z = Kalman_Filter(&PTZ_Kalman[2], robot.angle);
                else 
                    KF_Pos.Z = robot.angle;
                if(remote.v_x == 0 && remote.v_y == 0){//遥控器无输入
                    if(PID_Mode_Switch[0] == 0){//若切换了PID模式
                        PID_Mode_Switch[0] = Fuzzy_Rule_Init(13, 0.08, 11800, 40, 10, 1200, 0.2, &PTZ_followl_PID);
                        PID_Mode_Switch[1] = 0;
                    }
                    Fuzzy_Rule_Implementation(KF_Pos.Z, E_PTZ_BallHoop, &PTZ_followl_PID);
                }
                else if(remote.v_x != 0 && remote.v_y != 0){
                    if(PID_Mode_Switch[1] == 0){//若切换了PID模式
                        PID_Mode_Switch[1] = Fuzzy_Rule_Init(8, 0, 1000, 0, 30, 1200, 10, &PTZ_followl_PID);
                        PID_Mode_Switch[0] = 0;
                    }
                    Fuzzy_Rule_Implementation(KF_Pos.Z, E_PTZ_BallHoop, &PTZ_followl_PID);
                }
                Transformed_Ex.Z = -((PTZ_followl_PID.Output) / 2047.0f) * max_omega + remote.omega;//TOOD：正负判定  以2047为单位调的参，直接把输出映射
            }
            if(Circular_To_Absolute_motion == 1){//世界坐标系标志位
                float sin_ang, cos_ang;  
                arm_sin_cos_f32(reform(robot.angle + 180), &sin_ang, &cos_ang);//因为在家是处于进攻区，但是雷达反馈坐标是防守区而做出的妥协，临时加了个180
                Transformed_Ex.X =  remote.v_x * cos_ang + remote.v_y * sin_ang;
                Transformed_Ex.Y = -remote.v_x * sin_ang + remote.v_y * cos_ang;
                
                if (enable_offset && (remote.v_x != 0.0f || remote.v_y != 0.0f)){// 启用偏移自动校正 先不修改，因为R2没有这个也挺直
                    Closed_Loop_Angle -= remote.omega * 0.01f;
                    PID_Control2(robot.angle, Closed_Loop_Angle, &pid2);
                    Transformed_Ex.Z = pid2.pid_out;
                }
                else
                    Closed_Loop_Angle = robot.angle;
            }     
            /* if(Intercept_Flag == 1 && PTZ_followl == 1){
                Enemy_Hoop_Angle = RAD2ANGLE(-atan2f(Radar_t.Enemy_x - Side_Data.Defend_X_BallHoop,  Radar_t.Enemy_y - Side_Data.Defend_Y_BallHoop));
                float sin_ang, cos_ang; 
                arm_sin_cos_f32(Enemy_Hoop_Angle, &sin_ang, &cos_ang);
                Navigate_Point[0] = Radar_t.Enemy_x + Safe_Distance * sin_ang;
                Navigate_Point[1] = Radar_t.Enemy_y - Safe_Distance * cos_ang;
                Navigate_Point[2] = -RAD2ANGLE(atan2f(Navigate_Point[0] - Position_t.X, Navigate_Point[1] - Position_t.Y));
                arm_sin_cos_f32(Navigate_Point[2], &sin_ang, &cos_ang);
                World_EV[0] =-sin_ang * abs(Tx_RemoteData.rocker[3]);
                World_EV[1] = cos_ang * abs(Tx_RemoteData.rocker[3]);
                arm_sin_cos_f32(Position_t.Angle, &sin_ang, &cos_ang);
                Transformed_Ex.X +=  cos_ang * World_EV[0] + sin_ang * World_EV[1];//遥控期望叠加导航期望
                Transformed_Ex.Y += -sin_ang * World_EV[0] + cos_ang * World_EV[1];
                limit(Transformed_Ex.X, 500, -500);//仍未测试完成，先限制一下速度
                limit(Transformed_Ex.Y, 500, -500);
            }*/
            chassis.v_x = Transformed_Ex.X;
            chassis.v_y = Transformed_Ex.Y;
            chassis.omega = Transformed_Ex.Z; 
        }
        Last_PTZ_followl = PTZ_followl;

        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&chassis, sizeof(ChassisCtrl_t)); // TODO:发送底盘控制指令
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}



float reform(float X) { //钳制角度在-180到180 栈溢出会进入HardFault_Handler
    if(X > 180.f)
        return reform((X) - 360.f);
    if(X <-180.f)
        return reform((X) + 360.f);
    return X;
}	

