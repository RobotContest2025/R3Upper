#include "RemoteTask.h"
#include "MoveControl.h"
#include "Slope.h"
#include "Action.h"
#include "bezier.h"

// 动作执行队列
extern ChassisCtrl_t remote;
extern QueueHandle_t action_queue;
extern QueueHandle_t remote_semaphore;

// 运动模式控制全局变量
extern uint8_t Circular_To_Absolute_motion; // 运动控制
extern uint8_t PTZ_followl;                 // 是否锁定篮筐
extern uint8_t enable_offset;               // 是否启用偏移自动校正

UART_DataPack remoteRecv, last_remoteRecv;
JY61P_AccSensor_t jy61p, last_pack;
uint8_t remote_recv_buf[sizeof(UART_DataPack)];
char lv53_recv_buf[64] = {0};
uint8_t jy61p_recv[sizeof(JY61P_AccSensor_t)];
ChassisState_t chassis_state;
uint8_t chassis_state_recv[sizeof(ChassisState_t)];
extern RobotState_t robot;
PositionPack_Typedef position;
Position_Typedef Position_offest;
uint8_t position_recv_buf[sizeof(PositionPack_Typedef)];
extern float lv53_cur_dis;
extern int lv53_ball_dis;
extern float remote_rocker4;

extern int32_t claw_motor_target_pos;
extern int32_t claw_detach_pos;

RobotMode_Typedef RobotState = {.Mode = ROBOT_IDEL, .Robot_Init = 0, .Action_Finish = 0};

uint8_t interrupt_ready = 1;
uint8_t use_pc = 0;
// uint8_t dribble_ready_flag, dribble_flag;
// uint8_t throw_ready_flag, throw_flag;
// uint8_t ready_jump_flag, jump_flag;
// uint8_t defend_flag, pickup_flag;

static BezierLine bezier = {.p1_x = 0.660634f, .p1_y = 0.131222f, .p2_x = 0.846154f, .p2_y = 0.556561f}; // 摇杆贝塞尔曲线参数
float max_speed = 6.0f;                                                                           // 最大移动速度(m/s)
float max_omega = 180.0f;                                                                         // 最大角速度(度/s)
float cur_dir = 0.0f;

void RemoteTask(void *param)
{
    vTaskDelay(1000);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chassis_state_recv, sizeof(chassis_state_recv));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, remote_recv_buf, sizeof(remote_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, position_recv_buf, sizeof(position_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)lv53_recv_buf, sizeof(lv53_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jy61p_recv, sizeof(JY61P_AccSensor_t));

    vTaskDelay(10000);                                                                                       // 等待自检完成
    Action_t action = {.param = &RobotState.Robot_Init, .action_cb = ResetAction, .type = ACTION_TYPE_UNINTERRUPTABLE}; // 发送复位动作
    xQueueSend(action_queue, &action, portMAX_DELAY);
    RobotState.Mode = ROBOT_MOVE;

    uint32_t last_wake_time = HAL_GetTick();
    while (1)
    {
        if (xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) != pdTRUE) // 100ms未收到遥控器的数据，复位摇杆
        {
            remoteRecv.rocker[0] = 0;
            remoteRecv.rocker[1] = 0;
            remoteRecv.rocker[2] = 0;
        }

        float dt = (HAL_GetTick() - last_wake_time) * 0.001f;
        last_wake_time = HAL_GetTick();
        float vel[2], cur_target[2];
        vel[0] = ((float)(remoteRecv.rocker[0])) / 2047.0f;                                               // 归一化x轴速度
        vel[1] = ((float)(remoteRecv.rocker[1])) / 2047.0f;                                               // 归一化y轴速度
        remote.omega = BezierTransform(-(((float)(remoteRecv.rocker[2])) / 2047.0f) * max_omega, bezier); // 计算自转角速度
        remote_rocker4 = ((float)(remoteRecv.rocker[3])) / 2047.0f;                                       // 归一化第四个摇杆值

        float model = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
        if (model != 0.0f)
            cur_dir = atan2f(vel[1], vel[0]);

        model = BezierTransform(model, bezier);

        vel[0] = model * cosf(cur_dir) * max_speed;
        vel[1] = model * sinf(cur_dir) * max_speed;

        cur_target[0] = remote.v_x;
        cur_target[1] = remote.v_y;
        Vector2dSlope(vel, cur_target, 5.0f * dt); // 相邻两次速度变化量最大为5m/s*迭代时间,即加速度为5m/s^2

        remote.v_x = cur_target[0];
        remote.v_y = cur_target[1];

       if (remoteRecv.Key.Left_Switch_Down_or_Right) // 左边按键控制运动模式
            Circular_To_Absolute_motion = 1;
        else
            Circular_To_Absolute_motion = 0;

        if (remoteRecv.Key.Right_Switch_Down_or_Left) // 右边按键控制是否锁定篮筐
            PTZ_followl = 1;
        else
            PTZ_followl = 0;

        if (remoteRecv.Key.Left_Key_Up == 1 && last_remoteRecv.Key.Left_Key_Up == 0) // 不论车辆处于哪种模式，按下此按键均将机器人恢复到复位模式
        {
            RobotState.Robot_Init = 0;
            action.action_cb = ResetAction;
            action.type = ACTION_TYPE_UNINTERRUPTABLE;
            action.param = &RobotState.Robot_Init;
            xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
            RobotState.Mode = ROBOT_MOVE;
        }
        if(RobotState.Robot_Init == 1){
            if (remoteRecv.Key.Left_Key_Left == 1 && last_remoteRecv.Key.Left_Key_Left == 0) // 准备运球，运球动作按键
            {
                if (RobotState.Mode == ROBOT_READY_DRIBBLE && RobotState.Action_Finish == 1){//不进行准备动作的一次性检查，准备动作可以打断，不断被其他动作或再次执行此动作打断，若快速多次摁下按理来说会快速跳过过去已经执行完毕的流程，所以不怕多次执行
                    RobotState.Action_Finish = 0;
                    action.action_cb = DribbleAction2;
                    action.type = ACTION_TYPE_UNINTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_DRIBBLE;
                }
                else if (RobotState.Mode != ROBOT_LAUNCH && RobotState.Mode != ROBOT_JUMP && RobotState.Mode != ROBOT_DRIBBLE){ // 机器人处于允许进入运球模式的状态时，才允许进入准备运球模式，执行准备运球动作
                    RobotState.Action_Finish = 0;
                    action.action_cb = ReadyDribbleAction2;
                    action.type = ACTION_TYPE_INTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_READY_DRIBBLE;
                }
            }
            else if (remoteRecv.Key.Left_Key_Down == 1 && last_remoteRecv.Key.Left_Key_Down == 0) // 准备投球，投球动作按键
            {
                if (RobotState.Mode == ROBOT_READY_LAUNCH && RobotState.Action_Finish == 1){
                    RobotState.Action_Finish = 0;
                    action.action_cb = BackThrowAction;
                    action.type = ACTION_TYPE_UNINTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_LAUNCH;
                }
                else if (RobotState.Mode != ROBOT_LAUNCH && RobotState.Mode != ROBOT_JUMP && RobotState.Mode != ROBOT_DRIBBLE){ // 允许进入投球模式安全检查
                    RobotState.Action_Finish = 0;
                    action.action_cb = ReadyBackThrowAction;
                    action.type = ACTION_TYPE_INTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_READY_LAUNCH;
                }
            }
            else if (remoteRecv.Key.Left_Key_Right == 1 && last_remoteRecv.Key.Left_Key_Right == 0) // 准备扣篮，扣篮动作按键
            {
                if (RobotState.Mode == ROBOT_READY_JUMP && RobotState.Action_Finish == 1){
                    RobotState.Action_Finish = 0;
                    action.action_cb = JumpAction;
                    action.type = ACTION_TYPE_UNINTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_JUMP;
                }
                else if (RobotState.Mode != ROBOT_LAUNCH && RobotState.Mode != ROBOT_JUMP && RobotState.Mode != ROBOT_DRIBBLE){ // 允许进入扣篮模式安全检查
                    RobotState.Action_Finish = 0;
                    action.action_cb = ReadyJumpAction;
                    action.type = ACTION_TYPE_INTERRUPTABLE;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_READY_JUMP;
                }
            }
            else if (remoteRecv.Key.Right_Key_Up == 1 && last_remoteRecv.Key.Right_Key_Up == 0){ // 防守动作按键
                if (RobotState.Mode != ROBOT_LAUNCH && RobotState.Mode != ROBOT_JUMP && RobotState.Mode != ROBOT_DRIBBLE){ // 当前状态允许进入防守模式，不会自动退出，执行其它任意动作终止此动作
                    RobotState.Action_Finish = 0;
                    action.type = ACTION_TYPE_INTERRUPTABLE;
                    action.action_cb = DefendAction;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_DEFEND;
                }
            }
            else if (remoteRecv.Key.Right_Key_Left == 1 && last_remoteRecv.Key.Right_Key_Left == 0){
                if (RobotState.Mode != ROBOT_LAUNCH && RobotState.Mode != ROBOT_JUMP && RobotState.Mode != ROBOT_DRIBBLE){// 当前动作允许进入捡球模式，不会自动退出，执行其它任意动作终止此动作
                    RobotState.Action_Finish = 0;
                    action.type = ACTION_TYPE_INTERRUPTABLE;
                    action.action_cb = PickUpBallAction;
                    action.param = &RobotState.Action_Finish;
                    xQueueSend(action_queue, &action, pdMS_TO_TICKS(10));
                    RobotState.Mode = ROBOT_PICKUP;
                }
            }
            else if (remoteRecv.Key.Right_Key_Down == 1 && last_remoteRecv.Key.Right_Key_Down == 0) // 直接更改爪子开合状态
            {
                if (claw_motor_target_pos == 0)
                    claw_motor_target_pos = claw_detach_pos;
                else
                    claw_motor_target_pos = 0;
            }
            else if (remoteRecv.Key.Right_Key_Right == 1 && last_remoteRecv.Key.Right_Key_Right == 0) //
            {

            }
        }
        // 机器人状态更新
        if (RobotState.Action_Finish == 1 && (RobotState.Mode == ROBOT_DRIBBLE || RobotState.Mode == ROBOT_JUMP || RobotState.Mode == ROBOT_LAUNCH)) // 完成双步动作后，将机器人切换为移动模式
        {
            RobotState.Action_Finish = 0;
            RobotState.Mode = ROBOT_MOVE;
        }
        last_remoteRecv = remoteRecv; // 更新遥控器状态
    }
}

float actual_gn;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_DMAStop(huart);
    if (huart->Instance == USART3) // 遥控器
    {
        if (remote_recv_buf[0] == 0xAA) // 包头校验
        {
            memcpy(&remoteRecv, remote_recv_buf, sizeof(UART_DataPack));
            // HAL_UART_DMAStop(huart);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, remote_recv_buf, sizeof(remote_recv_buf));
            BaseType_t xHigherPriorityTaskWoken;
            xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if (huart->Instance == USART6) // 测距传感器1
    {
        char temp[16];
        int distance;
        int state;
        sscanf(lv53_recv_buf, "State;%d , %s %s\r\nd: %d mm", &state, temp, temp, &distance);
        if (state == 0)
        {
            if (distance != 0 && distance < 2000)
                lv53_cur_dis = ((float)distance) * 0.001f;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)lv53_recv_buf, sizeof(lv53_recv_buf));
    }
    else if (huart->Instance == UART5)
    {
        if (jy61p_recv[0] == 0x55 && jy61p_recv[1] == 0x51) // 加速度数据包正确
        {
            uint8_t sum = 0;
            for (int i = 0; i < sizeof(jy61p_recv) - 1; i++)
                sum += jy61p_recv[i];
            if (sum == jy61p_recv[sizeof(jy61p_recv) - 1])
                memcpy(&jy61p, jy61p_recv, sizeof(jy61p_recv));

            float _gn = jy61p.z / 32768.0f * 16.0f;
            if (_gn > 1.5f || _gn < 0.5f) // 数据不可信,使用上一次的包
                jy61p = last_pack;
            else
                last_pack = jy61p;

            actual_gn = jy61p.z / 32768.0f * 16.0f;
        }
        else if (jy61p_recv[0] == 0x55 && jy61p_recv[1] == 0x52) // 角速度数据
        {
            uint8_t sum = 0;
            for (int i = 0; i < sizeof(jy61p_recv) - 1; i++)
                sum += jy61p_recv[i];
            if (sum == jy61p_recv[sizeof(jy61p_recv) - 1])
                robot.omega = ((((int16_t)jy61p_recv[7]) << 8) | jy61p_recv[6]) / (32768.0f / 2000.0f);
        }
        if (!use_pc) // 如果PC未连接，那么使用陀螺仪的角度数据
        {
            if (jy61p_recv[0] == 0x55 && jy61p_recv[1] == 0x53)
            {
                uint8_t sum = 0;
                for (int i = 0; i < sizeof(jy61p_recv) - 1; i++)
                    sum += jy61p_recv[i];
                if (sum == jy61p_recv[sizeof(jy61p_recv) - 1])
                    robot.angle = ((((int16_t)jy61p_recv[7]) << 8) | jy61p_recv[6]) / (32768.0f / 180.0f);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jy61p_recv, sizeof(JY61P_AccSensor_t));
    }
    else if(huart->Instance==UART4)     //底盘状态反馈
    {
        if(chassis_state_recv[0] ==0x1B)    //包头校验
        {
            memcpy(&chassis_state, chassis_state_recv, sizeof(chassis_state_recv));
            robot.v_x= chassis_state.v_x;
            robot.v_y= chassis_state.v_y;
            robot.omega = chassis_state.omega;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chassis_state_recv, sizeof(chassis_state_recv));
        }
    }
    else if (huart->Instance == USART2)
    {
        if (position_recv_buf[0] == 0x2B)
        {
            use_pc = 1;
            memcpy(&position, position_recv_buf, sizeof(position_recv_buf));
            robot.x = position.x;
            robot.y = position.y;
            if (position.angle / 180.f + Position_offest.angle - robot.angle > 180)
                Position_offest.angle -= 360;
            else if (position.angle / 180.f + Position_offest.angle - robot.angle < -180)
                Position_offest.angle += 360;
            robot.angle = position.angle / 180.f + Position_offest.angle;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, position_recv_buf, sizeof(position_recv_buf));
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) // 收到错误帧处理
{
    if (huart->Instance == USART3)
    {
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, remote_recv_buf, sizeof(remote_recv_buf));
    }
    else if (huart->Instance == USART6)
    {
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)lv53_recv_buf, sizeof(lv53_recv_buf));
    }
    else if (huart->Instance == USART2)
    {
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, position_recv_buf, sizeof(position_recv_buf));
    }
    else if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jy61p_recv, sizeof(JY61P_AccSensor_t));
    }
}
