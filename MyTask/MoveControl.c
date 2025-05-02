#include "MoveControl.h"
#include "PID_old.h"
#include "FastCal.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim14;

// 需要在RemoteControl中分配按键控制这两个变量
uint8_t control_mode = 0;   // 运动控制
uint8_t lock_to_basket = 0; // 是否锁定篮筐

ChassisCtrl_t chassis, remote;
RobotState_t robot;
Point_t basket = {.x = 0.0f, .y = 3.1f};
Point_t basket_t;

PID2 pid = {.Kp = 0.5f, .Ki = 0.0f, .Kd = 0.0f, .limit = 100.0f, .output_limit = 0.5f}; // PID控制器结构体
float s_k = -0.5f,t_k = -0.0f;
float hand_posture_angle=0.0f,hand_posture_x=0.1f,hand_posture_y=0.2f;
float angle_err;

void MoveControlTask(void *param)
{
    Matrix_t *gnd_rob_matrix = (Matrix_t *)CreateMatrix(3, 3); // 将地面坐标系转移到机器人坐标系的转移矩阵
    Matrix_t *rob_hand_matrix = (Matrix_t *)CreateMatrix(3, 3); // 将机器人坐标系转移到发射机构坐标系的转移矩阵

    Matrix_t *basket_pos = (Matrix_t *)CreateMatrix(3, 1);
    SetElement(basket_pos, 0, 0, basket.x);
    SetElement(basket_pos, 1, 0, basket.y);
    SetElement(basket_pos, 2, 0, 1.0f);

    Matrix_t *temp = (Matrix_t *)CreateMatrix(3, 1);    //存储中间运算结果的矩阵
    Matrix_t *temp1 = (Matrix_t *)CreateMatrix(3, 1);
    Matrix_t *temp2 = (Matrix_t *)CreateMatrix(3, 3);

    FillMatrix3d(temp2, Fastcos(hand_posture_angle), -Fastsin(hand_posture_angle), hand_posture_x, // 发射机构坐标系到机器人坐标系的转移矩阵
                        Fastsin(hand_posture_angle), Fastcos(hand_posture_angle), hand_posture_y,
                        0.0f, 0.0f, 1.0f);
    MatrixInverse(rob_hand_matrix, temp2);       //矩阵求逆，得到机器人坐标系到发射机构坐标系的转移矩阵

    TickType_t last_wake_time = xTaskGetTickCount(); // 获取当前时间戳
    while (1)
    {
        float cos_angle=Fastcos(robot.angle);
        float sin_angle=Fastsin(robot.angle);

        // 填写坐标变换矩阵元素
        FillMatrix3d(temp2, cos_angle, -sin_angle, (float)robot.x, // 机器人坐标系下的物体到场地坐标系的转移矩阵
                            sin_angle, cos_angle, (float)robot.y,
                            0.0f, 0.0f, 1.0f);
        MatrixInverse(gnd_rob_matrix, temp2);       // 矩阵求逆，得到场地坐标系到机器人坐标系的转移矩阵

        MatrixProduct(temp1, gnd_rob_matrix, basket_pos);        // 求得篮筐在机器人坐标系下的位置
        MatrixProduct(temp, rob_hand_matrix, temp1);             // 求得发射机构坐标系下的篮筐位置
        basket_t.x = GetElement(temp, 0, 0);
        basket_t.y = GetElement(temp, 1, 0);
        // 计算机器人的转动
        if (lock_to_basket) // 机器人需要面朝篮筐
        {
            float angle_err = RAD2ANGLE(atan2f(basket_t.y, basket_t.x)); // 求得球筐与车在当前坐标系下的角度（理想情况下应该为0）
            if(angle_err<-90.0f)    //机器人锁定角度大约为90度左右（y轴朝前），需要将角度从-180~180转换到-90~270之间，保证在PID控制器的作用下以劣弧转动并对齐目标
                angle_err+=360.0f;

            float basket_dis = sqrtf(basket_t.x * basket_t.x + basket_t.y * basket_t.y); // 求得篮筐到投射机构的距离

            if(basket_dis>=10.0f)
            {
                pid.Kp=0.1f;
                pid.Kd=0.1f;
                PID_Control2(angle_err, 90.0f, &pid);
                chassis.omega = pid.pid_out - RAD2ANGLE((robot.iner_vx / basket_dis)) + remote.omega;
            }
            else if (basket_dis > 0.5f)
            {
                pid.Kp=s_k*(basket_dis-10.0f);
                pid.Kd=t_k*(basket_dis-10.0f);
                PID_Control2(angle_err, 90.0f, &pid);
                chassis.omega = pid.pid_out - RAD2ANGLE((robot.iner_vx / basket_dis)) + remote.omega; // 绕篮筐移动时自旋角速度的计算（观测误差PID输出+角速度前馈+用户强制输入）
            }
            else      // 当距离比较小时，为了防止robot.v_x / basket_dis震荡，不再使用该项。同时固定PID参数
            {
                pid.Kp=1.0f;   //距离足够小，使用固定的一个PID参数
                pid.Kd=1.0f;
                PID_Control2(angle_err,0.0f,&pid);
                chassis.omega = pid.pid_out+ remote.omega;
            }
        }
        else
            chassis.omega = remote.omega;

        
        // 计算机器人的移动
        if (control_mode) // 使用场地坐标系进行控制
        {
            SetElement(temp1, 0, 0, remote.v_x);
            SetElement(temp1, 1, 0, remote.v_y);
            SetElement(temp1, 2, 0, 0.0f);
            MatrixProduct(temp, gnd_rob_matrix, temp1); // 将场地坐标系下的期望速度转换为车体坐标系下的期望速度
            chassis.v_x = GetElement(temp, 0, 0);
            chassis.v_y = GetElement(temp, 1, 0);
        }
        else // 使用车体坐标系进行控制
        {
            chassis.v_x = remote.v_x;
            chassis.v_y = remote.v_y;
        }

        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&chassis, sizeof(ChassisCtrl_t)); // TODO:发送底盘控制指令
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
