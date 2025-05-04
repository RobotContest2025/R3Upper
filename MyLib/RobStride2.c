#include "RobStride2.h"

void RobStrideInit(RobStride_t *device, CAN_HandleTypeDef *hcan, uint32_t id, RobStrideMode mode, RobStrideType type)
{
    device->hcan = hcan;
    device->motor_id = id;
    device->host_id = HOST_ID;
    device->type = type;
}

uint32_t RobStrideEnable(RobStride_t *device)
{
    uint8_t buf[8]={0};
    return RobStrideSend(device, (3<<24)|(device->host_id<<8)|(device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideDisable(RobStride_t *device,uint8_t clear_error)
{
    uint8_t buf[8]={clear_error};
    return RobStrideSend(device, (4<<24)|(device->host_id<<8)|(device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideResetAngle(RobStride_t *device)
{
    uint8_t buf[8]={1};
    return RobStrideSend(device, (6<<24)|(device->host_id<<8)|(device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideGet(RobStride_t *device,uint16_t cmd)
{
    uint8_t buf[8]={0};
    *((uint16_t*)buf)=cmd;
		uint32_t Extid=(17<<24)|(device->host_id<<8)|(device->motor_id);
    return RobStrideSend(device,Extid, buf); 						// 发送使能命令
}

uint32_t RobStrideSetMode(RobStride_t *device,RobStrideMode mode)
{
    uint8_t buf[8]={0};
    buf[0]=(uint8_t)PARAM_RUN_MODE;
    buf[1]=(uint8_t)(PARAM_RUN_MODE>>8);
    buf[4]=(uint8_t)mode;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideMotionControl(RobStride_t *device,float torque,float target_rad,float target_omega)   //运控模式
{
    //TODO:尚未实现的功能
    return 0;
}

uint32_t RobStrideTorqueControl(RobStride_t *device,float req)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_IQ_REF;
    *((float*)&buf[4])=req;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideSpeedControl(RobStride_t *device,float vel)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_SPD_REF;
    *((float*)&buf[4])=vel;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStridePositionControl(RobStride_t *device,float pos)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_LOC_REF;
    *((float*)&buf[4])=pos;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideSetVelPID(RobStride_t *device,float kp,float ki)
{
    uint8_t buf[8];
    uint32_t ret;
    *((uint32_t*)(&buf[0]))=PARAM_SPD_KP;
    *((float*)&buf[4])=kp;
    ret=RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);

    *((uint32_t*)(&buf[0]))=PARAM_SPD_KI;
    *((float*)&buf[4])=ret|RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
    return 0;
}

uint32_t RobStrideSetLocPID(RobStride_t *device,float kp)
{
    uint8_t buf[8];
    *((uint32_t*)(&buf[0]))=PARAM_LOC_KP;
    *((float*)&buf[4])=kp;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideSetCurPID(RobStride_t *device,float kp,float ki)
{
    //TODO:设置电流环PID（一般不需要，所以就不写了）
    return 0;
}

uint32_t RobStrideSetVelLimit(RobStride_t *device,float vel)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_LIMIT_SPD;
    *((float*)&buf[4])=vel;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideSetCurLimit(RobStride_t *device,float cur)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_LIMIT_CUR;
    *((float*)&buf[4])=cur;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideSetTorqueLimit(RobStride_t *device,float torque)
{
    uint8_t buf[8]={0};
    *((uint16_t*)&buf[0])=PARAM_LIMIT_TORQUE;
    *((float*)&buf[4])=torque;
    return RobStrideSend(device, (18<<24)|(device->host_id<<8)|(device->motor_id), buf);
}

uint32_t RobStrideRecv_Handle(RobStride_t *device, CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf)
{
    if (hcan->Instance != device->hcan->Instance)
        return 0;
    if (((ID >> 8) & 0x00FF) != device->motor_id)
        return 0;
    uint32_t type = ID >> 24;
    if (type == 0)
    {
        // TODO:尝试获取设备信息以检查通信
    }
    else if (type == 2) // 存储反馈的数据
    {
        if (device->type == RobStride_04)
        {
            device->state.feedback = ID >> 16;
            device->state.rad = (float)(((DEPACK_U8_TO_U16_FLIP(buf[0], buf[1])) - 32767) * 4.0 * M_PI / 32767);
            device->state.omega = (float)(((DEPACK_U8_TO_U16_FLIP(buf[2], buf[3])) - 32767) * 15 / 32767);
            device->state.torque = (float)(((DEPACK_U8_TO_U16_FLIP(buf[4], buf[5])) - 32767) * 120 / 32767);
            device->state.temperature = DEPACK_U8_TO_U16_FLIP(buf[6], buf[7]) * 0.1f;
        }
        else if (device->type == RobStride_01)
        {
            device->state.feedback = ID >> 16;
            device->state.rad = (float)(((DEPACK_U8_TO_U16_FLIP(buf[0], buf[1])) - 32767) * 4.0 * M_PI / 32767);
            device->state.omega = (float)(((DEPACK_U8_TO_U16_FLIP(buf[2], buf[3])) - 32767) * 44 / 32767);
            device->state.torque = (float)(((DEPACK_U8_TO_U16_FLIP(buf[4], buf[5])) - 32767) * 17 / 32767);
            device->state.temperature = DEPACK_U8_TO_U16_FLIP(buf[6], buf[7]) * 0.1f;
        }
    }
    else if (type == 21) // 复制错误信息到电机状态结构体
    {
        device->state.error = *((uint64_t *)buf);
    }
    else if (type == 17)    //请求读取单个参数时的处理
    {
        switch (DEPACK_U8_TO_U16(buf[0], buf[1]))
        {
        case PARAM_RUN_MODE: // 当前控制模式
            device->param.run_mode = buf[4];
            break;
        case PARAM_LIMIT_TORQUE: // 力矩限制
            device->param.torque_limit = *((float *)&buf[4]);
            break;
        case PARAM_CUR_KP:
            device->param.cur_kp = *((float *)&buf[4]);
            break;
        case PARAM_CUR_KI:
            device->param.cur_ki = *((float *)&buf[4]);
            break;
        case PARAM_CUR_FILT_GAIN:
            device->param.cur_filt_gain = *((float *)&buf[4]);
            break;
        case PARAM_LIMIT_SPD:
            device->param.limit_spd = *((float *)&buf[4]);
            break;
        case PARAM_LIMIT_CUR:
            device->param.limit_cur = *((float *)&buf[4]);
            break;
        case PARAM_LOC_KP:
            device->param.loc_kp = *((float *)&buf[4]);
            break;
        case PARAM_SPD_KP:
            device->param.spd_kp = *((float *)&buf[4]);
            break;
        case PARAM_SPD_KI:
            device->param.spd_ki = *((float *)&buf[4]);
            break;
        default:
            break;
        }
    }
    return 1;
}

uint32_t RobStrideSend(RobStride_t *device, uint32_t ExtID, uint8_t *buf)
{
    CAN_TxHeaderTypeDef head;
    uint32_t mailbox;
		head.StdId = 0;
    head.ExtId = ExtID;
    head.IDE = CAN_ID_EXT;
    head.RTR = CAN_RTR_DATA;
    head.DLC = 8;
    head.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(device->hcan, &head, buf, &mailbox);
}
