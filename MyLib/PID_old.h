/**
 * @file    PID.h
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块头文件
 */

#ifndef _PIDOLD2_H_
#define _PIDOLD2_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 限幅宏函数
 * @param IN 限幅变量
 * @param MAX 最大值
 * @param MIN 最小值
 */
#ifndef limit
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX
#endif
/**
 * @brief 标准位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_now;    //!<@brief 当前误差
    float error_last;   //!<@brief 上一次误差
    float error_inter;  //!<@brief 误差积分
    float pid_out;      //!<@brief PID输出
    float output_limit;
} PID2;

/**
 * @brief 带史密斯预估器的位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_now;    //!<@brief 当前误差
    float error_inter;  //!<@brief 误差积分
    float pid_out;      //!<@brief PID输出
} PID_Smis2;

/**
 * @brief 增量式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float error_now;    //!<@brief 当前误差
    float error_next;   //!<@brief 上一次误差
    float error_last;   //!<@brief 上上次误差
    float increament;   //!<@brief PID增量
} PID_ADD2;

/**
 * @brief 标准位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 */
void PID_Control2(float current, float expected, PID2 *data);



/**
 * @brief 增量式PID
 * @param[in] current 实际值
 * @param[in] expect 期望值
 * @param[in] parameter PID参数
 * @return PID增量
 */
float PID_Increment2(float current, float expect, PID_ADD2 *parameter);

#ifdef __cplusplus
}
#endif

#endif
