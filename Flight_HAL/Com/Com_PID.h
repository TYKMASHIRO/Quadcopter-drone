#ifndef __COM_PID_H
#define __COM_PID_H

#include "main.h"

typedef volatile struct
{
    float desired;   // 期望值
    float prevError; // 上次偏差
    float integ;     // 误差积分累加值
    float kp;        // p参数
    float ki;        // i参数
    float kd;        // d参数
    float measured;  // 实际测量值
    float out;       // pid输出
} PidObject;

void ResetPID(PidObject ** pidObjects, uint8_t len);

void PID_Update(PidObject *pid, float dt);

void CasecadePID(PidObject *pidAngle, PidObject *pidRate, float dt);

#endif
