/**
 * @file    CANDrive.h
 * @author  yao
 * @date    1-May-2020
 * @brief   CAN底层驱动模块头文件
 * @note
 *  针对cube生成的代码编写！
 *  如果使用实时系统添加全局宏定义__USE_RTOS启用发送临界区保护
 */

#ifndef _CANDrive_H_
#define _CANDrive_H_

#include "stm32f4xx_hal.h"

#if !defined(CAN1) && defined(CAN)
#define CAN1 CAN
#endif


/**
 * @brief 按照通常设置初始化CAN滤波器
 * @param hcan CAN handle Structure definition
 */
void CanFilter_Init(CAN_HandleTypeDef* hcan);

/**
 * @brief CAN发送标准/拓展帧数据
 * @param hcan CAN句柄
 * @param[in] StdId 标准帧ID ExTId 拓展帧ID
 * @param[in] msg 数据数组,长度为8
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef CAN_Send_StdDataFrame(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *msg);
HAL_StatusTypeDef CAN_Send_ExTDataFrame(CAN_HandleTypeDef *hcan, uint32_t ExTId, uint8_t *msg);
/**
 * @brief CAN读取数据
 * @param hcan CAN句柄
 * @param[out] buf 数据缓冲区
 * @return 标准帧ID或拓展帧ID
 */
uint32_t CAN_Receive_DataFrame(CAN_HandleTypeDef *hcan, uint8_t *buf);

#endif
