#include "Com_PID.h"

void ResetPID(PidObject **pidObjects, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        pidObjects[i]->prevError = 0;
        pidObjects[i]->integ = 0;
        pidObjects[i]->out = 0;
    }
}

void PID_Update(PidObject *pid, float dt)
{
    float temp_err; // 本次偏差
    float dri;
    /* 1、计算偏差值 */
    temp_err = pid->desired - pid->measured;
    /* 2、计算积分： 偏差累积和 */
    pid->integ += temp_err * dt;
    /* 3、计算微分： 偏差的变化率 */
    dri = (temp_err - pid->prevError) / dt;
    /* 4、结果保存到out里，本次偏差值保存到 “上次偏差” 字段 */
    pid->out = pid->kp * temp_err + pid->ki * pid->integ + pid->kd * dri;
    pid->prevError = temp_err;
}

void CasecadePID(PidObject *pidAngle, PidObject *pidRate, float dt)
{
    /* 1、角度外环进行PID处理 */
    PID_Update(pidAngle, dt);
    /* 2、外环的输出，赋值给内环的期望值 */
    pidRate->desired = pidAngle->out;
    /* 3、对内环进行PID计算 */
    PID_Update(pidRate, dt);
}
