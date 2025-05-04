/**
 * @file Odrive.h
 * @author Liu yuanzhao
 * @date 31-Dec-2024
 * @brief ODrive电机驱动板控制
 * @note 适用于ODrive v3.6 参考https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
 */

#ifndef __ODRIVE_H__
#define __ODRIVE_H__

#include "stm32f4xx_hal.h"
#include "CANDrive.h"
//#include "Canbuffer.h"
#include <stdlib.h>
#include <string.h>

#define ODRIVE_CMD_READ     (CAN_RTR_REMOTE<<5)
#define ODRIVE_CMD_WRITE    (CAN_RTR_DATA<<5)

/**
 * @defgroup ODrive_Cmd ODrive命令
 * @{
 */
#define ODRIVE_GET_HEARTBEAT                        0x001   //心跳消息包
#define ODRIVE_SET_ESTOP                            0x002   //急停
#define ODRIVE_GET_MOTOR_ERROR                      0x003   //获取电机误差
#define ODRIVE_GET_ENCODER_ERROR                    0x004   //获取编码器误差
#define ODRIVE_GET_SENSORLESS_ERROR                 0x005   //获取无传感器误差
#define ODRIVE_SET_ID                               0x006     //设置电机的ID
#define ODRIVE_SET_AXIS_STATE                       0x007     //设置轴的状态(传入宏)
//#define ODRIVE_0x008                              0x008                     //不可用
#define ODRIVE_GET_ENCODER_ESTIMATE                 0x009   //获取编码器估计的位置和速度
#define ODRIVE_GET_ENCODER_CNT                      0x00A   //获取编码器精确计数值
#define ODRIVE_SET_MODE                             0x00B     //设置控制器的控制模式和输入模式(传入宏)
#define ODRIVE_SET_INPUT_POSITION                   0x00C     //位置控制模式时，输入位置信息
#define ODRIVE_SET_INPUT_VELOCITY                   0x00D     //速度控制模式时，输入速度信息
#define ODRIVE_SET_INPUT_TORQUE                     0x00E     //力矩控制时，输入控制力矩
#define ODRIVE_SET_VELOCITY_AND_CURRENT_LIMIT       0x00F     //设置速度和电流限制
#define ODRIVE_SET_TRAG_VELOCITY_LIMIT              0x010     //电机斜坡控制下，设置速度限制
#define ODRIVE_SET_TRAG_CURRENT_LIMIT               0x011     //电机斜坡控制下，设置电流限制
#define ODRIVE_SET_TRAG_ACCEL_LIMIT                 0x012     //电机斜坡控制下，设置加速度和减速度限制
#define ODRIVE_SET_TRAG_INERTIA                     0x013     //电机斜坡控制下，设置模拟的惯性大小
#define ODRIVE_GET_TORQUE_CURRENT                   0x014   //获取转矩电流目标值和当前值
#define ODRIVE_GET_SENSORLESS_ESTIMATES             0x015   //获取无传感器估计的位置和速度
#define ODRIVE_REBOOT                               0x016     //重启ODrive
#define ODRIVE_GET_BUS_VOLTAGE_AND_CURRENT          0x017   //获取总线电流和电压
#define ODRIVE_CLEAR_ERROR                          0x018     //清除控制器错误
#define ODRIVE_SET_LINEAR_COUNT                     0x019     //设置编码器计数值
#define ODRIVE_SET_POSITION_GAIN                    0x01A     //设置位置模式下，控制器的参数P
#define ODRIVE_SET_VELOCITY_GAINS                   0x01B     //设置速度环增益（在位置模式和速度模式下均有效）
#define ODRIVE_GET_ADC_VOLTAGE                      0x01C     //读取ADC的电压值
#define ODRIVE_GET_CONTROLLER_ERROR                 0x01D     //获取控制器错误

/**
 * @}
 */


/**
 * @defgroup ODrive_ControlMode 设置ODrive的控制模式
 * @{
 */
#define ODRIVE_SET_WORKMODE_VOLTAGE_CONTROL         0x00
#define ODRIVE_SET_WOORKMODE_TORQUE_CONTROL         0x01
#define ODRIVE_SET_WORKMODE_VELOCITY_CONTROL        0x02
#define ODRIVE_SET_WORKMODE_POSITION_CONTROL        0x03

/**
 * @}
 */


/**
 * @defgroup ODrive_InputMode 设置ODrive的输入模式
 * @{
 */

//ODrive设置控制器的输入模式
#define ODRIVE_SET_INPUTMODE_INACTIVE               (0x00<<4)        //不使用输入，电机输出为发出该命令前的最后一个输出设置
#define ODRIVE_SET_INPUTMODE_PASSTHROUGH            (0x01<<4)        //直接将输入作用到目标值（适用于所有控制模式）
#define ODRIVE_SET_INPUTMODE_VEL_RAMP               (0x02<<4)        //将转速以斜坡控制方式渐变到目标值（仅在速度控制下有效）
//#define ODRIVE_SET_INPUTMODE_POS_FILTER           0x03
//#define ODRIVE_SET_INPUTMODE_MIX_CHANNELS         0x04
//#define ODRIVE_SET_INPUTMODE_TRAP_TRAJ            0x05
#define ODRIVE_SET_INPUTMODE_TORQUE_RAMP            (0x06<<4)        //将输入力矩通过斜坡渐变到目标值
#define ODRIVE_SET_INPUTMODE_MIRROR                 (0x07<<4)        //精确地复制另一个电机的转动（仅支持双板）

/**
 * @}
 */


/**
 * @defgroup ODrive_Axis_State ODrive设置电机的状态
 * @{
 */
#define ODRIVE_SET_AXIS_STATE_UNDEFINED                     0  // 未定义状态，通常不使用
#define ODRIVE_SET_AXIS_STATE_IDLE                          1  // 空闲状态，电机停止运行，PWM 输出为零
#define ODRIVE_SET_AXIS_STATE_STARTUP_SEQUENCE              2  // 启动序列，用于轴初始化（常在上电后运行）
#define ODRIVE_SET_AXIS_STATE_FULL_CALIBRATION_SEQUENCE     3  // 完整校准序列，包括电机和编码器校准
#define ODRIVE_SET_AXIS_STATE_MOTOR_CALIBRATION             4  // 电机校准，检测电机参数（电阻、电感等）
#define ODRIVE_SET_AXIS_STATE_SENSORLESS_CONTROL            5  // 无传感器控制模式（不使用编码器）
#define ODRIVE_SET_AXIS_STATE_ENCODER_INDEX_SEARCH          6  // 编码器索引搜索，用于校准支持索引的编码器
#define ODRIVE_SET_AXIS_STATE_ENCODER_OFFSET_CALIBRATION    7  // 编码器偏移校准，校正电机和编码器位置偏差
#define ODRIVE_SET_AXIS_STATE_CLOSED_LOOP_CONTROL           8  // 闭环控制模式，根据目标位置、速度或电流运行
#define ODRIVE_SET_AXIS_STATE_LOCKIN_SPIN                   9  // 自旋锁定，用于无刷电机启动过程（无传感器控制时）

/**
 * @}
 */


#pragma pack(1)

#if 1   //（这是使用#if1是为了方便在VSCode中将代码段折叠起来）
//ODrive心跳包数据0x001
typedef struct
{
    uint32_t axisError;
    uint8_t axisState;
    uint8_t encoderErrorFlag;
    uint8_t controllerErrorFlag;
    uint8_t trajectoryFinishedFlag;
}ODriveHeartbeadMsg;

//位置，速度数据0x009
typedef struct
{
    float position;
    float velocity;
}ODrivePosVelEstimateMsg;

//编码器数据0x00A
typedef struct
{
    int32_t cnt;
    int32_t cpr;
}ODriveEocoderMsg;

//ODrive工作模式0x00B
typedef struct
{
    int32_t controllerMode;
    int32_t inputMode;
}ODriveWorkModeMsg;

//ODrive位置模式，数据结构体0x00C
typedef struct
{
    float position;
    int16_t velocity;       //单位0.001rev/s
    int16_t torque;         //单位0.001Nm
}ODrivePositionMsg;

//ODrive速度模式，数据结构体0x00D
typedef struct
{
    float velocity;        //单位rev/s
    float torque;         //单位Nm
}ODriveVelocityMsg;

//ODrive力矩模式,数据结构体0x00E
typedef struct Odrive
{
    float torque;
}ODriveTorqueMsg;

//ODrive速度和电流限制0x00F
typedef struct
{
    float velocityLimit;
    float currentLimit;
}ODriveLimit;

typedef struct
{
    float velocity;
    float accel;
    float decel;
    float trajInertia;
}ODriveTrajLimit;

//ODrive力矩电流0x014
typedef struct
{
    float torqueCurrentTarget;
    float torqueCurrentNow;
}ODriveTorqueCurrentnMsg;

typedef struct
{
    float posGain;                                    //ODrive位置环的位置增益设置0x01A
    float velocityGain;                               //ODrive速度环增益设置0x01B
    float velocityIntegratorGain;                     //ODrive速度环积分项增益设置0x01B
}ODriveVelGainMsg;

//ODrive总线上的电压和电流0x01C
typedef struct
{
    float Voltage;
    float Current;
}ODrivePowerMsg;

//ODrive编码器计数值0x019
typedef struct
{
    int32_t cnt;
}ODriveLinearCount;

#endif //基本数据包定义


/*
ODriveWorkModeMsg workModeSet;              //设置电机工作模式和输入模式
ODrivePositionMsg positionControlSet;       //设置位置控制模式下的控制数据
ODriveVelocityMsg velocityControlSet;       //设置速度控制模式下的控制数据
ODriveTorqueMsg torqueControlSet;           //设置力矩控制模式下的控制数据
ODriveLimit sysLimitSet;                    //设置电流和力矩限制
ODriveTrajLimit trajLimitSet;               //启用电机斜坡控制时，对斜坡的限制条件
ODriveLinearCount encoderSet;               //设置编码器编码数
*/


//ODrive设备属性结构体
typedef struct
{
    CAN_HandleTypeDef *hcan;                    //使用的CAN外设的句柄

    uint32_t motorID;                           //电机ID

    ODriveHeartbeadMsg heartBeatGet;            //得到心跳包消息
    ODrivePosVelEstimateMsg posVelEstimateGet;  //得到当前的速度和位置浮点数据
    ODriveEocoderMsg encoderGet;                //得到编码器数据
    ODriveTorqueCurrentnMsg torqueCurrentGet;   //得到力矩电流的目标值和当前值
    ODrivePowerMsg powerGet;                    //得到当前总线的电压和电流
    uint8_t rawData[8];
}ODrive;

#pragma pack()


uint32_t ODriveSendOrReceiveData(ODrive *odrive,uint8_t cmd,void *param);
uint8_t ODriveRecvServe(ODrive *odrive,uint32_t ID,uint8_t *buf);


uint32_t ODriveSetControlMode(ODrive *odrive,uint8_t controlmode_inputmode);
uint32_t ODriveSetVelocity(ODrive *odrive,float velocity,float toqrue);
uint32_t ODriveSetPosition(ODrive *odrive,float position,float velocity,float torque);
uint32_t ODriveSetLimit(ODrive *odrive,float velocity,float current);
uint32_t ODriveSetTragLimit(ODrive *odrive,float max_velocity,float max_acc,float max_dec,float inertia);
//uint32_t ODriveSetGainsParam(ODrive *odrive,ODriveVelGainMsg *param);

/**
 * @brief 设置电机的状态
 * @param axisState 可以是 @ref ODrive_Axis_State 中的一个值
 */
#define ODriveSetAxisState(odrive,axisState)        ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_AXIS_STATE,&axisState)

/**
 * @brief 设置电机的力矩
 */
#define ODriveSetTorque(odrive,torque)              ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_INPUT_TORQUE,&torque)

/**
 * @brief 获取当前ADC得到的电压
 */
#define ODriveGetADCVoltage(odrive)                 ODriveSendOrReceiveData(odrive,ODRIVE_CMD_READ|ODRIVE_GET_ADC_VOLTAGE,NULL)

/**
 * @brief 获取控制器错误信息
 */
#define ODriveGetControlError(odrive)               ODriveSendOrReceiveData(odrive,ODRIVE_CMD_READ|ODRIVE_GET_CONTROLLER_ERRO,NULL)

/**
 * @brief 获取位置和速度（浮点型）
 */
#define ODriveGetEncoderEstimate(odrive)            ODriveSendOrReceiveData(odrive,ODRIVE_CMD_READ|ODRIVE_GET_ENCODER_ESTIMATE,NULL)

/**
 * @brief 直接获取编码器的值
 */
#define ODriveGetEncoderCNT(odrive)                 ODriveSendOrReceiveData(odrive,ODRIVE_CMD_READ|ODRIVE_GET_ENCODER_CNT,NULL)

/**
 * @brief 获取总线上的电压和电流等能量信息
 */
#define ODriveGetPowerInfo(odrive)                  ODriveSendOrReceiveData(odrive,ODRIVE_CMD_READ|ODRIVE_GET_BUS_VOLTAGE_AND_CURRENT,NULL)

/**
 * @brief 重启ODrive驱动板
 */
#define ODriveReboot(odrive)                        ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_REBOOT,NULL)

/**
 * @brief 清除错误消息
 */
#define ODriveClearError(odrive)                    ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_CLEAR_ERROR,NULL)

/**
 * @brief 设置编码器的值
 */
#define ODriveSetLinearCnt(odrive,cnt)              ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_LINEAR_COUNT,&cnt)

#define ODriveSetGainsParam(odrive,param)           ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_POSITION_GAIN,&param->posGain);\
                                                    ODriveSendOrReceiveData(odrive,ODRIVE_CMD_WRITE|ODRIVE_SET_VELOCITY_GAINS,&param->velocityGain)

#endif
