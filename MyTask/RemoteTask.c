#include "RemoteTask.h"
#include "MoveControl.h"
#include "Slope.h"

 extern uint8_t lv53_recv_buf[64];
 extern LV53_Sensor_t lv53_sensor;
// extern uint8_t lv53_recv_buf_chass[64];
// extern LV53_Sensor_t lv53_sensor_chass;
PositionPack_Typedef position_pack;
extern RobotState_t robot;

//动作执行队列
extern ChassisCtrl_t remote;
extern QueueHandle_t action_queue;
extern QueueHandle_t remote_semaphore;

//运动模式控制全局变量
extern uint8_t control_mode;   // 运动控制
extern uint8_t lock_to_basket; // 是否锁定篮筐
extern uint8_t enable_offset;  // 是否启用偏移自动校正

UART_DataPack remoteRecv, last_remoteRecv;
uint8_t remote_recv_buf[sizeof(UART_DataPack)];
// uint8_t jy61p_recv_buf[sizeof(JY61P_t)];

uint32_t interrupt_ready = 1;

float max_speed = 2.0f;  // 最大移动速度(m/s)
float max_omega = 72.0f; // 最大角速度(度/s)

void RemoteTask(void *param)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, remote_recv_buf, sizeof(remote_recv_buf));
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jy61p_recv_buf, sizeof(jy61p_recv_buf));
    //  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lv53_recv_buf_chass, sizeof(lv53_recv_buf_chass));
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lv53_recv_buf, sizeof(lv53_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)&position_pack, sizeof(position_pack));

    Action_t action = {.param = NULL, .action_cb = ResetAction}; // 发送复位动作
    //xQueueSend(action_queue, &action, portMAX_DELAY);

    uint32_t last_wake_time = HAL_GetTick();
    while (1)
    {
        if (xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(100)) != pdTRUE)     //超时未收到遥控器的数据，复位摇杆
        {
            remoteRecv.rocker[0] = 0x7FF;
            remoteRecv.rocker[1] = 0x7FF;
            remoteRecv.rocker[3] = 0x7FF;
        }

        float dt = (HAL_GetTick() - last_wake_time) * 0.001f;
        last_wake_time = HAL_GetTick();
        float vel[2];
        vel[1] = ((float)(remoteRecv.rocker[0] - 0x7FF)) / 2047.0f * max_speed; // x轴速度
        vel[0] = ((float)(remoteRecv.rocker[1] - 0x7FF)) / 2047.0f * max_speed; // y轴速度
        remote.omega = (((float)(remoteRecv.rocker[3] - 0x7FF)) / 2047.0f) * max_omega;
				float cur_target[2];
				cur_target[0]=remote.v_x;
				cur_target[1]=remote.v_y;
        Vector2dSlope(vel, cur_target, 3.0f * dt); // 相邻两次速度变化量最大为3m/s*迭代时间
				remote.v_x=cur_target[0];
				remote.v_y=cur_target[1];


        if (remoteRecv.Key.Left_Key_Up == 1 && last_remoteRecv.Key.Left_Key_Up == 0) //执行测试动作1，动作类型为不可打断
        {
            action.action_cb=TestAction;
            action.type=ACTION_TYPE_INTERRUPTABLE;
            action.param=NULL;
            xQueueSend(action_queue,&action,pdMS_TO_TICKS(10));
        }
        else if (remoteRecv.Key.Left_Key_Down == 1 && last_remoteRecv.Key.Left_Key_Down == 0) // 急停模式
        {
            action.action_cb=LaunchInVel;
            action.type=ACTION_TYPE_INTERRUPTABLE;
            action.param=NULL;
            xQueueSend(action_queue,&action,pdMS_TO_TICKS(10));
        }
        else if (remoteRecv.Key.Left_Key_Left == 1 && last_remoteRecv.Key.Left_Key_Left == 0) // 云台跟随模式
        {
			action.action_cb=FastJump;
            action.type=ACTION_TYPE_INTERRUPTABLE;
            action.param=NULL;
            xQueueSend(action_queue,&action,pdMS_TO_TICKS(10));
        }
        else if (remoteRecv.Key.Left_Key_Right == 1 && last_remoteRecv.Key.Left_Key_Right == 0) // 发射态和运球态切换
        {
						action.action_cb=JumpInVel;
            action.type=ACTION_TYPE_INTERRUPTABLE;
            action.param=NULL;
            xQueueSend(action_queue,&action,pdMS_TO_TICKS(10));
        }
        else if (remoteRecv.Key.Right_Key_Up == 1 && last_remoteRecv.Key.Right_Key_Up == 0) // 开始执行一次运球
        {
					action.action_cb=DebugAction;
            action.type=ACTION_TYPE_INTERRUPTABLE;
            action.param=NULL;
            xQueueSend(action_queue,&action,pdMS_TO_TICKS(10));
        }
        else if (remoteRecv.Key.Right_Key_Left == 1 && last_remoteRecv.Key.Right_Key_Left == 0) // 底盘下降，准备运球和发射，也可以执行发射操作
        {

        }
        else if (remoteRecv.Key.Right_Key_Down == 1 && last_remoteRecv.Key.Right_Key_Down == 0) // 执行发射序列
        {

        }
        else if (remoteRecv.Key.Right_Key_Right == 1 && last_remoteRecv.Key.Right_Key_Right == 0)
        {

        }
        last_remoteRecv = remoteRecv; // 更新遥控器状态
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, remote_recv_buf, sizeof(remote_recv_buf));
    }
}


float chassis_v = 0;
float _velocity = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    float temp_value;
    if (huart->Instance == UART4) // 遥控器
    {
        if (remote_recv_buf[0] == 0xAA) // 包头校验
        {
            memcpy(&remoteRecv, remote_recv_buf, sizeof(UART_DataPack));
            BaseType_t higherPriorityTaskWoken;

            BaseType_t xHigherPriorityTaskWoken;
            xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            //TODO:FeedDog()
        }
    }
        else if (huart->Instance == USART6) // 测距传感器1
        {
            char temp[16];
            int distance;
            int state;
            sscanf((char *)lv53_recv_buf, "State;%d , %s %s\r\nd: %d mm", &state, temp, temp, &distance);
            if (state == 0)
                lv53_sensor.distance = distance;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6, lv53_recv_buf, sizeof(lv53_recv_buf));
            // TODO:FeedDog();
        }
    else if (huart->Instance == USART6) // 测距传感器2
    {
        /*static uint8_t first = 1;
        char temp[16];
        int distance;
        int state;

        sscanf((char *)lv53_recv_buf_chass, "State;%d , %s %s\r\nd: %d mm", &state, temp, temp, &distance);
        if (state == 0)
            lv53_sensor_chass.distance = distance;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lv53_recv_buf_chass, sizeof(lv53_recv_buf_chass));
        if (first)
        {
            first = 0;
            chassis_v = 0;
        }
        else
        {
            temp_value = 1.0f * (lv53_sensor_chass.distance - lv53_sensor_chass.last_distance) / 0.02f;
            chassis_v = Kalman_Filter(&kalman, temp_value);
            _velocity = temp_value;
        }
        lv53_sensor_chass.last_distance = lv53_sensor_chass.distance;*/
    }
    else if (huart->Instance == UART5)
    {
        robot.x = position_pack.X / 1000.0f;
        robot.y = position_pack.Y / 1000.0f;
        robot.angle = position_pack.Angle;
        robot.iner_vx = position_pack.Body_X_Velocity;
        robot.iner_vy = position_pack.Body_Y_Velocity;
        robot.omega = position_pack.Z_Velocity;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)&position_pack, sizeof(position_pack));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, remote_recv_buf, sizeof(remote_recv_buf));
    // else if (huart->Instance == UART5)
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jy61p_recv_buf, sizeof(jy61p_recv_buf));
    //  else if (huart->Instance == USART6)
    //      HAL_UARTEx_ReceiveToIdle_DMA(&huart6, lv53_recv_buf, sizeof(lv53_recv_buf));
    //  else if (huart->Instance == USART3)
    //      HAL_UARTEx_ReceiveToIdle_DMA(NULL, lv53_recv_buf, sizeof(lv53_recv_buf));
}
