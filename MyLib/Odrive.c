#include "Odrive.h"

//命令对应的CAN数据包的长度
const static uint8_t odriveCmdLength[0x01D+1]={0,8,0,8,4,4,4,4,0,8,8,8,8,8,4,4,0,4,8,4,8,8,0,8,0,4,4,8,4,4};

/**
 * @brief 设置电机工作模式
 * @param odrive ODrive句柄
 * @param controlmode_inputmode 应当为控制模式和输入模式的或运算，即一个 @ref ODrive_ControlMode 和 @ref 和一个 @ref ODrive_InputMode 的或运算
 * @return CAN数据发送情况
 * @note 无
 */
uint32_t ODriveSetControlMode(ODrive *odrive,uint8_t controlmode_inputmode)
{
    ODriveWorkModeMsg temp;
    temp.controllerMode=controlmode_inputmode&0x0F;
    temp.inputMode=controlmode_inputmode>>4;
    return ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_MODE,&temp);
}

/**
 * @brief 速度模式下，设置期望速度
 * @param odrive ODrive句柄
 * @param velocity 期望速度
 * @param current 前馈力矩
 * @return CAN数据发送情况
 * @note 只能在速度模式下使用
 */
uint32_t ODriveSetVelocity(ODrive *odrive,float velocity,float toqrue)
{
    ODriveVelocityMsg temp;
    temp.torque=toqrue;
    temp.velocity=velocity;
    return ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_INPUT_VELOCITY,&temp);
}

/**
 * @brief 位置模式下，设置期望位置
 * @param odrive ODrive句柄
 * @param position 期望的位置
 * @param velocity 速度设置
 * @param current 电流设置
 * @return CAN数据发送情况
 * @note 只能在位置模式下使用
 */
uint32_t ODriveSetPosition(ODrive *odrive,float position,float velocity,float torque)
{
    const float k_v=1.0f/0.001f,k_t=1.0f/0.001f;
    ODrivePositionMsg temp;
    temp.position=position;
    temp.velocity=(int16_t)(velocity*k_v);
    temp.torque=(int16_t)(torque*k_t);
    return ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_INPUT_POSITION,&temp);
}

/**
 * @brief 设置电流和速度限制
 * @param odrive ODrive句柄
 * @param velocity 最大速度限制
 * @param current 最大电流限制
 * @return CAN数据发送情况
 */
uint32_t ODriveSetLimit(ODrive *odrive,float velocity,float current)
{
    ODriveLimit temp;
    temp.currentLimit=current;
    temp.velocityLimit=velocity;
    return ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_VELOCITY_AND_CURRENT_LIMIT,&temp);
}

/**
 * @brief 设置使用斜坡控制时，斜坡的各个参数
 * @param odrive ODrive句柄
 * @param max_velocity 斜坡控制时，电机的最大速度（位置环）
 * @param max_acc 斜坡控制时，电机的最大加速度
 * @param max_dcc 斜坡控制时，电机的最大减速度
 * @param inertia 斜坡控制时，模拟的惯量
 * @return CAN数据的发送情况
 */
uint32_t ODriveSetTragLimit(ODrive *odrive,float max_velocity,float max_acc,float max_dec,float inertia)
{
    ODriveTrajLimit temp;
    temp.velocity=max_velocity;
    temp.accel=max_acc;
    temp.decel=max_dec;
    temp.trajInertia=inertia;
    ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_TRAG_VELOCITY_LIMIT,&temp);
    ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_TRAG_ACCEL_LIMIT,((uint8_t*)&temp+sizeof(float)));
    return ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_TRAG_INERTIA,((uint8_t*)&temp+3*sizeof(float)));
}


/**
 * @brief ODrive驱动板数据发送或请求函数
 * @details 从ODrive句柄获取信息没通过指定的CAN向指定的电机发送命令，并根据命令内容可选地发送数据
 * @param odrive ODrive句柄
 * @param cmd 发送的命令，应该是
 * @param paramAddr 要发送的数据[是否有用取决于命令内容]
 * @return CAN数据发送情况
 */
uint32_t ODriveSendOrReceiveData(ODrive *odrive,uint8_t cmd,void *paramAddr)
{
    CAN_TxHeaderTypeDef head;
    uint32_t TxMailbox;
    head.ExtId=0;
    head.IDE=CAN_ID_STD;
    head.TransmitGlobalTime=DISABLE;
    head.StdId=(cmd&0x1F)|(odrive->motorID<<5);
    head.RTR=cmd>>5;
    if(head.RTR==CAN_RTR_DATA)
        head.DLC=odriveCmdLength[cmd&0x1F];
    else
        head.DLC=0;

    #if 0
    return CAN_Transmit(odrive->hcan,&head,paramAddr);
    #else
    return HAL_CAN_AddTxMessage(odrive->hcan,&head,(uint8_t*)(paramAddr),&TxMailbox);
    #endif
}

/** 
* @brief ODrive驱动板数据接收函数
* @details 放置在CAN中断处理函数中，应当先调用CAN_Receive_DataFrame()获取ID和数据，之后将ID和数据传入
* @param odrive ODrive句柄
* @param ID CAN数据帧ID
* @param buf CAN收到的数据
* @return 接收到了意外的数据帧返回2，正常处理数据返回1，没有进行任何数据处理返回0
*/
uint8_t ODriveRecvServe(ODrive *odrive,uint32_t ID,uint8_t *buf)     //接收消息函数，应当放在CAN数据接收中断处理函数中
{
    if (ID>>5==odrive->motorID)    //确认当前消息是该电机需要的消息
    {
        switch(ID&0x1F)
        {
            case ODRIVE_GET_HEARTBEAT:
                memcpy(&odrive->heartBeatGet,buf,sizeof(ODriveHeartbeadMsg));
                break;
            case ODRIVE_GET_ENCODER_ESTIMATE:
                memcpy(&odrive->posVelEstimateGet,buf,sizeof(ODrivePosVelEstimateMsg));
                break;
            case ODRIVE_GET_ENCODER_CNT:
                memcpy(&odrive->encoderGet,buf,sizeof(ODriveEocoderMsg));
                break;
            case ODRIVE_GET_TORQUE_CURRENT:
                memcpy(&odrive->torqueCurrentGet,buf,sizeof(ODriveTorqueCurrentnMsg));
                break;
            case ODRIVE_GET_BUS_VOLTAGE_AND_CURRENT:
                memcpy(&odrive->powerGet,buf,sizeof(ODrivePowerMsg));
                break;
            default:
                memcpy(odrive->rawData,buf,8);
                return 2;
        }
        return 1;
    }
    return 0;
}
