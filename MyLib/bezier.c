#include "bezier.h"


float bezier_get_t(float x,float p1_x,float p2_x)
{
    // 由于 x0 = 0, x3 = 1, 则贝塞尔 x(t) 表达式为：
    // x(t) = 3*p1_x*t*(1-t)^2 + 3*p2_x*t^2*(1-t) + t^3
    // 将其写为多项式形式：
    // x(t) = a*t^3 + b*t^2 + c*t, 其中:
    float a = 1.0f + 3.0f * p1_x - 3.0f * p2_x;
    float b = 3.0f * p2_x - 6.0f * p1_x;
    float c = 3.0f * p1_x;
    // 方程写为: a*t^3 + b*t^2 + c*t - x = 0
    float d = -x;
    
    // 若 a 非零，按三次方程处理；否则退化为二次或一次方程
    if (fabsf(a) < 1e-6f) {
        if (fabsf(b) < 1e-6f) { // 线性： c*t + d = 0
            return -d / c;
        } else { // 二次： b*t^2 + c*t + d = 0
            float disc = c * c - 4.0f * b * d;
            if (disc < 0) disc = 0;
            float sqrt_disc = sqrtf(disc);
            float t1 = (-c + sqrt_disc) / (2.0f * b);
            float t2 = (-c - sqrt_disc) / (2.0f * b);
            return (t1 > t2 ? t1 : t2);
        }
    }
    
    // 将方程归一化： t^3 + (b/a)*t^2 + (c/a)*t + (d/a) = 0
    float bb = b / a;
    float cc = c / a;
    float dd = d / a;
    // 令 t = y - bb/3 消去二次项，则标准消去二次项的三次方程为：
    // y^3 + P*y + Q = 0, 其中：
    float P = cc - (bb * bb) / 3.0f;
    float Q = (2.0f * bb * bb * bb - 9.0f * bb * cc + 27.0f * dd) / 27.0f;
    
    // 计算判别式 delta = (Q/2)^2 + (P/3)^3
    float halfQ = Q / 2.0f;
    float delta = halfQ * halfQ + (P / 3.0f) * (P / 3.0f) * (P / 3.0f);
    
    float t_out = -1.0f; // 最终返回的 t 值
    
    if (delta >= 0.0f) {
        // delta >= 0: 存在唯一实根（或重根）
        float sqrt_delta = sqrtf(delta);
        float u = cbrtf(-halfQ + sqrt_delta);
        float v = cbrtf(-halfQ - sqrt_delta);
        float y = u + v;
        // 回代： t = y - (bb/3)
        t_out = y - bb / 3.0f;
    } else {
        // delta < 0: 三个不同的实根
        float r = 2.0f * sqrtf(-P / 3.0f);
        // 为防止由于数值误差导致 acosf 参数超出 [-1,1]，需作限幅
        float temp = sqrtf(-P / 3.0f);
        float acos_arg = -halfQ / (temp * temp * temp);
        if (acos_arg > 1.0f)  acos_arg = 1.0f;
        if (acos_arg < -1.0f) acos_arg = -1.0f;
        float theta = acosf(acos_arg);
        
        // 三个解： y_k = r * cos((theta + 2π*k)/3), k = 0,1,2
        float y0 = r * cosf(theta / 3.0f);
        float y1 = r * cosf((theta + 2.0f * M_PI) / 3.0f);
        float y2 = r * cosf((theta + 4.0f * M_PI) / 3.0f);
        
        // 回代： t = y - (bb/3)
        float t0 = y0 - bb / 3.0f;
        float t1 = y1 - bb / 3.0f;
        float t2 = y2 - bb / 3.0f;
        
        // 多个解中选择较大的一个
        t_out = t0;
        if (t1 > t_out) t_out = t1;
        if (t2 > t_out) t_out = t2;
    }
    return t_out;
}

float bezier_get_y(float t,float x,float p1_y,float p2_y)
{
    return 3.0f*t*(1.0f-t)*(1.0f-t)*p1_y+3*t*t*(1.0f-t)*p2_y+t*t*t;
}
