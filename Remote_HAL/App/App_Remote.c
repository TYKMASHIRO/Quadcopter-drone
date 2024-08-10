#include "App_Remote.h"

struct _Rc rc;
struct _Offset offset;
struct _Filter Filter_Thr, Filter_Pitch, Filter_Roll, Filter_Yaw;

extern uint16_t ADC_Value[4];

/**
 * @description: 取最近N次的平均值
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Window_Filter(struct _Rc *rc)
{
    static uint16_t count = 0;
    /* 1、丢掉旧数据 */
    Filter_Thr.sum -= Filter_Thr.old[count];
    Filter_Pitch.sum -= Filter_Pitch.old[count];
    Filter_Roll.sum -= Filter_Roll.old[count];
    Filter_Yaw.sum -= Filter_Yaw.old[count];

    /* 2、加上新数据:新数据要加到sum里，同时也要存到old里 */
    Filter_Thr.old[count] = rc->THR;
    Filter_Pitch.old[count] = rc->PIT;
    Filter_Roll.old[count] = rc->ROL;
    Filter_Yaw.old[count] = rc->YAW;

    Filter_Thr.sum += Filter_Thr.old[count];
    Filter_Pitch.sum += Filter_Pitch.old[count];
    Filter_Roll.sum += Filter_Roll.old[count];
    Filter_Yaw.sum += Filter_Yaw.old[count];

    /* 3、求平均值 */
    rc->THR = Filter_Thr.sum / Filter_Num;
    rc->PIT = Filter_Pitch.sum / Filter_Num;
    rc->ROL = Filter_Roll.sum / Filter_Num;
    rc->YAW = Filter_Yaw.sum / Filter_Num;

    /* 4、判断索引值的范围 */
    count++;
    if (count >= 10)
    {
        count = 0;
    }
}

/**
 * @description: 长按按键，进行摇杆中点校准（油门最低校准）
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Offset(struct _Rc *rc)
{
    static uint16_t key_count = 0;
    static uint32_t sum_thr = 0, sum_pit = 0, sum_roll = 0, sum_yaw = 0;
    static uint16_t count = 0;
    /* 判断按键长按 */
    if (!READ_KEY_LEFT_X)
    {
        printf("start offset...\r\n");
        key_count++;
        if (key_count >= 20) // 暂时按照main函数while中的延时来调整
        {
            /* 长按了，开始校准中点 =》 偏差值 = 50次的和/50 - 1500 */
            if (count == 0)
            {
                /* count=0时，进行初始化，包括累加值、偏差值 */
                sum_thr = 0;
                sum_pit = 0;
                sum_roll = 0;
                sum_yaw = 0;
                offset.THR = 0;
                offset.PIT = 0;
                offset.ROL = 0;
                offset.YAW = 0;
                /* count+1，下次不再进初始化 */
                count = 1;
                return;
            }
            else
            {
                /* 开始累加50次、计数（2-51） */
                count++;
                sum_thr += rc->THR;
                sum_pit += rc->PIT;
                sum_roll += rc->ROL;
                sum_yaw += rc->YAW;
            }
            if (count == 51)
            {
                count--;                             // 进入这里count已经是51，实际累加了50次
                offset.THR = sum_thr / count - 1000; // 油门特殊，最低校准
                offset.PIT = sum_pit / count - 1500;
                offset.ROL = sum_roll / count - 1500;
                offset.YAW = sum_yaw / count - 1500;

                /* 获取到偏差值后，再去清零count、key_count */
                count = 0;
                key_count = 0;
            }
        }
    }
}

/**
 * @description: 最大值、最小值限幅
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Limit(struct _Rc *rc)
{
    rc->THR = (rc->THR < 1000) ? 1000 : rc->THR;
    rc->THR = (rc->THR > 2000) ? 2000 : rc->THR;

    rc->PIT = (rc->PIT < 1000) ? 1000 : rc->PIT;
    rc->PIT = (rc->PIT > 2000) ? 2000 : rc->PIT;

    rc->ROL = (rc->ROL < 1000) ? 1000 : rc->ROL;
    rc->ROL = (rc->ROL > 2000) ? 2000 : rc->ROL;

    rc->YAW = (rc->YAW < 1000) ? 1000 : rc->YAW;
    rc->YAW = (rc->YAW > 2000) ? 2000 : rc->YAW;
}

/**
 * @description: 中点限幅：认为在中间附近，就是要中值
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Limit(struct _Rc *rc)
{
    /* 考虑到油门不会回弹，手动拉到完全的中间很难，范围给大一点 */
    if (rc->THR > 1450 && rc->THR < 1550)
    {
        rc->THR = 1500;
    }

    /* 其他三个摇杆会自动回中，范围可以给小一点 */
    if (rc->PIT > 1490 && rc->PIT < 1510)
    {
        rc->PIT = 1500;
    }
    if (rc->ROL > 1490 && rc->ROL < 1510)
    {
        rc->ROL = 1500;
    }
    if (rc->YAW > 1490 && rc->YAW < 1510)
    {
        rc->YAW = 1500;
    }
}

// /**
//  * @description: 摇杆扫描
//  * @return {*}
//  */
void App_Remote_Stick_Scan()
{
    /* 1、将ADC的值转换为 1000-2000的范围，并且处理极性 */
    /*
        0: 右摇杆的左右
        1: 右摇杆的上下
        2: 左摇杆的左右
        3: 左摇杆的上下
     */
    rc.THR = 1000 + (uint16_t)(0.25f * ADC_Value[1]) - offset.THR;
    rc.PIT = 1000 + (uint16_t)(0.25f * ADC_Value[3]) - offset.PIT;
    rc.ROL = 1000 + (uint16_t)(0.25f * ADC_Value[2]) - offset.ROL;
    rc.YAW = 2000 - (uint16_t)(0.25f * ADC_Value[0]) - offset.YAW;

    /* 滑动窗口滤波：变化变得平缓 */
    App_Remote_Window_Filter(&rc);
    /* 中点校准：油门最低校准，其他中点校准 */
    // App_Remote_Mid_Offset(&rc);
    /* 最大最小限幅 */
    App_Remote_Limit(&rc);
    /* 中点限幅 */
    App_Remote_Mid_Limit(&rc);
}

/**
 * @description: 左下四个按键微调俯仰（前进、后退）、横滚（左移、右移）
 * @return {*}
 */
void App_Remote_KeyPress()
{
    /*
        整个标志位变量=
        if(按键按下&& 标志位==0)
        {
            延迟一下
            if(按键按下 && 标志位==0)
            {
                处理逻辑；
                标志=1；
            }

        }
        if(按键松开 && 标志位=1)
        {
            标志位=0；
        }
     */

    /* 1、判断哪个按键按下，对应的去微调 */
    if (!READ_KEY_U)
    {
        /* 前进按键按下，微调pitch */
        offset.PIT = offset.PIT - 10; // 前进用减（让pitch变大=减掉的偏差值变小）
    }
    /* 为什么不用else if:用户有可能多个按键同时按，都判定生效的话，单独if */
    if (!READ_KEY_D)
    {
        /* 前进按键按下，微调pitch */
        offset.PIT = offset.PIT + 10; // 后退用加（让pitch变小=减掉的偏差值变大）
    }
    if (!READ_KEY_L)
    {
        /* 左边按键按下，微调roll */
        offset.ROL = offset.ROL + 10; // 左移用加（让roll变小=减掉的偏差值变大）
    }
    if (!READ_KEY_R)
    {
        /* 左边按键按下，微调roll */
        offset.ROL = offset.ROL - 10; // 右移用减（让roll变大=减掉的偏差值变小）
    }
}

/**
 * @description: 根据通信协议，封装要发送给飞控的数据
 * @return {*}
 */
void App_Remote_RemoteData(uint8_t *remote_send)
{
    uint8_t index = 0;
    uint32_t check_sum = 0;
    /* 协议：帧头+功能字+长度+数据+校验和 */
    /* 1 帧头 */
    remote_send[index++] = 0xAA;
    remote_send[index++] = 0xAF;
    /* 2 功能字 */
    remote_send[index++] = 0x03;
    /* 3 长度 */
    remote_send[index++] = 0x0; // 先给个0，后面计算再填进去
    /* 4 遥控数据 */
    /* 4.1 摇杆数据: THR -》 YAW -》 ROL -》 PIT，每个都拆为 高8位、低8位 */
    remote_send[index++] = (uint8_t)(rc.THR >> 8);
    remote_send[index++] = (uint8_t)rc.THR;
    remote_send[index++] = (uint8_t)(rc.YAW >> 8);
    remote_send[index++] = (uint8_t)rc.YAW;
    remote_send[index++] = (uint8_t)(rc.ROL >> 8);
    remote_send[index++] = (uint8_t)rc.ROL;
    remote_send[index++] = (uint8_t)(rc.PIT >> 8);
    remote_send[index++] = (uint8_t)rc.PIT;
    /* 4.2 辅助通道aux1-aux6，现在没用，可以不写，也可以写 */
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;

    /* 5 重新计算数据长度 */
    remote_send[3] = index - 4;

    /* 6 校验和 */
    for (uint8_t i = 0; i < index; i++)
    {
        check_sum += remote_send[i];
    }
    remote_send[index++] = (uint8_t)(check_sum >> 24);
    remote_send[index++] = (uint8_t)(check_sum >> 16);
    remote_send[index++] = (uint8_t)(check_sum >> 8);
    remote_send[index++] = (uint8_t)(check_sum);
}
