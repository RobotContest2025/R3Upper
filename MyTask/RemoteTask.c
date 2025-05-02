#include "RemoteTask.h"
#include "MoveControl.h"

extern uint8_t lv53_recv_buf[64];
extern LV53_Sensor_t lv53_sensor;
extern uint8_t lv53_recv_buf_chass[64];
extern LV53_Sensor_t lv53_sensor_chass;
PositionPack_Typedef position_pack;
extern RobotState_t robot;

extern ChassisCtrl_t remote;

extern QueueHandle_t action_queue;
extern QueueHandle_t remote_semaphore;
extern QueueHandle_t action_deal_mutex;

UART_DataPack remoteRecv, last_remoteRecv;
uint8_t remote_recv_buf[sizeof(UART_DataPack)];

uint32_t interrupt_ready = 1;
kalman_filter_t kalman;

float max_speed = 2.0f; // 最大移动速度(m/s)
float max_omega = 72.0f; // 最大角速度(度/s)

void RemoteTask(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, remote_recv_buf, sizeof(remote_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lv53_recv_buf_chass, sizeof(lv53_recv_buf_chass));
    //HAL_UARTEx_ReceiveToIdle_DMA(&huart6, lv53_recv_buf, sizeof(lv53_recv_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*)&position_pack, sizeof(position_pack));
    Action_t action = {.param = NULL, .action_cb = ResetAction};
    xQueueSend(action_queue, &action, portMAX_DELAY);
    uint8_t dribble_launch_flag = 1;
    uint8_t ready_dribble = 0;
    uint8_t mainbody_is_low = 0;
    uint32_t continue_dribble = 1;

    kalman_Init(&kalman, 2.0f, 600.0f);

    while (1)
    {
        xSemaphoreTake(remote_semaphore, portMAX_DELAY);
        // TODO:接收遥控器控制量并转换为国际单位
        remote.v_y = ((float)(remoteRecv.rocker[1] - 0x7FF)) / 2047.0f * max_speed;
        remote.v_x = ((float)(remoteRecv.rocker[0] - 0x7FF)) / 2047.0f * max_speed;
        remote.omega = (((float)(remoteRecv.rocker[2] - 0x7FF)) / 2047.0f) * max_omega;

        // vTaskDelay(pdMS_TO_TICKS(10));
        if (remoteRecv.Key.Left_Key_Up == 1 && last_remoteRecv.Key.Left_Key_Up == 0) // 手动遥控模式
        {
            action.action_cb = TestAction;
            xQueueSend(action_queue, &action, 0);
        }
        else if (remoteRecv.Key.Left_Key_Down == 1 && last_remoteRecv.Key.Left_Key_Down == 0) // 急停模式
        {
            action.action_cb = TestAction2;
            xQueueSend(action_queue, &action, 0);
        }
        else if (remoteRecv.Key.Left_Key_Left == 1 && last_remoteRecv.Key.Left_Key_Left == 0) // 云台跟随模式
        {
        }
        else if (remoteRecv.Key.Left_Key_Right == 1 && last_remoteRecv.Key.Left_Key_Right == 0) // 发射态和运球态切换
        {
            if (dribble_launch_flag)
            {
                interrupt_ready = 0; // 强行终止上一个准备动作
                action.param = &interrupt_ready;
                action.action_cb = ReadyDribbleAction;
            }
            else
            {
                interrupt_ready = 0;
                action.param = &interrupt_ready;
                action.action_cb = ReadyLaunchAction;
            }
            dribble_launch_flag = !dribble_launch_flag;
            xQueueSend(action_queue, &action, 0); // 将该动作序列函数提交给操作电机的任务
        }
        else if (remoteRecv.Key.Right_Key_Up == 1 && last_remoteRecv.Key.Right_Key_Up == 0) // 开始执行一次运球
        {
            ready_dribble = 1;   // Debug，调试完成后要删除
            mainbody_is_low = 1; // Debug，调试完成后要删除
            if (ready_dribble && mainbody_is_low)
            {
                continue_dribble = 1;
                action.param = &continue_dribble; // 传入外部标志位的地址，方便通过遥控器强行终止运球动作序列
                action.action_cb = DribbleAction;
                xQueueSend(action_queue, &action, 0);
            }
        }
        else if (remoteRecv.Key.Right_Key_Left == 1 && last_remoteRecv.Key.Right_Key_Left == 0) // 底盘下降，准备运球和发射，也可以执行发射操作
        {
            mainbody_is_low = 1;
            action.action_cb = MainBodyDeclineAction;
            xQueueSend(action_queue, &action, 0);
        }
        else if (remoteRecv.Key.Right_Key_Down == 1 && last_remoteRecv.Key.Right_Key_Down == 0) // 执行发射序列
        {
            mainbody_is_low = 1;                           // Debug，完成后删除
            dribble_launch_flag = 0;                       // Debug，完成后删除
            if ((!dribble_launch_flag) && mainbody_is_low) // 执行完ReadyDribble后，dribble_launch_flag=0，并且主体还要位于低位
            {
                mainbody_is_low = 0;
                action.action_cb = LaunchAction;
                xQueueSend(action_queue, &action, 0);
            }
        }
        else if (remoteRecv.Key.Right_Key_Right == 1 && last_remoteRecv.Key.Right_Key_Right == 0)
        {
            continue_dribble = 0; // 取消运球
        }

        last_remoteRecv = remoteRecv; // 更新遥控器状态
    }
}

float chassis_v = 0;
float _velocity = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    float temp_value;
    if (huart->Instance == UART4) // 遥控器
    {
        if (remote_recv_buf[0] == 0x5A) // 包头校验
        {
            memcpy(&remoteRecv, remote_recv_buf, sizeof(UART_DataPack));
            BaseType_t higherPriorityTaskWoken;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, remote_recv_buf, sizeof(remote_recv_buf));

            BaseType_t xHigherPriorityTaskWoken;
            xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            // TODO:将遥控信号转发给底盘控制的主板
            // TODO:FeedDog()
        }
    }
//    else if (huart->Instance == USART6) // 测距传感器1
//    {
//        char temp[16];
//        int distance;
//        int state;
//        sscanf((char *)lv53_recv_buf, "State;%d , %s %s\r\nd: %d mm", &state, temp, temp, &distance);
//        if (state == 0)
//            lv53_sensor.distance = distance;
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, lv53_recv_buf, sizeof(lv53_recv_buf));
//        // TODO:FeedDog();
//    }
    else if (huart->Instance == UART5) // 测距传感器2
    {
        static uint8_t first = 1;
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
        lv53_sensor_chass.last_distance = lv53_sensor_chass.distance;
    }
		else if(huart->Instance == USART6)
		{
			robot.x=position_pack.X/1000.0f;
			robot.y=position_pack.Y/1000.0f;
			robot.angle=-position_pack.Angle;
			robot.omega=position_pack.Z_Velocity;
			robot.iner_vx=position_pack.Body_X_Velocity;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*)&position_pack, sizeof(position_pack));
		}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_recv_buf, sizeof(remote_recv_buf));
    else if (huart->Instance == UART5)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lv53_recv_buf, sizeof(lv53_recv_buf));
    //else if (huart->Instance == USART6)
    //    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, lv53_recv_buf, sizeof(lv53_recv_buf));
    else if (huart->Instance == USART3)
        HAL_UARTEx_ReceiveToIdle_DMA(NULL, lv53_recv_buf, sizeof(lv53_recv_buf));
}
