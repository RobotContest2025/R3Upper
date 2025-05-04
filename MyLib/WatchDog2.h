/**
 * @file WatchDog2.h
 * @author Liu yuanzhao
 * @brief 看门狗模块
 * @note 基于FreeRTOS
 * @date 2025-3-24
 * @version 1.1
 * @par 修改日志:
 * 1.1: 完善了注释，增加可读性
 */


#ifndef __WATCHDOG2_H__
#define __WATCHDOG2_H__

#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "stm32f4xx_hal.h"

#define WATCHDOG_MODE_ONCE 0x07
#define WATCHDOG_MODE_REPEAT 0x06

//看门狗每秒轮询频率（Hz） 注意：该值严禁大于1000
#define WATCHDOG_FREQUENCY 10

typedef void (*WatchDogCb_t)(void *user_data);

typedef struct
{
    uint32_t id;
    uint32_t last_wake_up_time_ms;
    uint32_t timeout_ms;
    void *user_data;        //用户需要传入给回调函数的参数
    WatchDogCb_t watchdog_cb;   //看门狗超时回调
    uint8_t state;          //看门狗状态，第一位表示模式，第二位表示是否暂时挂起，第三位表示是否启用看门狗
} WatchDog2_t;


/**
 * @brief 添加看门狗查询任务
 * @param watchdog_cb 看门狗超时回调函数
 * @param timeout_ms 超时时间
 * @param user_data 用户数据
 * @param mode 看门狗模式 
 *          - WATCHDOG_MODE_ONCE 超时后看门狗只执行一次，然后自动挂起 
 *          - WATCHDOG_MODE_REPEAT 超时后看门狗回调在每一次查询中都会被执行
 * @return 看门狗id
 */
uint32_t AddWatchDog(WatchDogCb_t watchdog_cb, uint32_t timeout_ms, void *user_data, uint32_t mode);


/**
 * @brief 喂狗
 * @param id 看门狗id
 * @return 1 成功；0 失败
 */
uint32_t FeedDog(uint32_t id);


/**
 * @brief 暂停看门狗
 * @param id 看门狗id
 * @return 1 成功；0 失败
 */
uint32_t DisableDog(uint32_t id);

/**
 * @brief 启用看门狗
 * @param id 看门狗id
 * @return 1 成功；0 失败
 */
uint32_t EnableDog(uint32_t id);


/**
 * @brief 删除看门狗
 * @param id 看门狗id
 * @return 1 成功；0 失败
 */
uint32_t DeleteDog(uint32_t id);

#endif
