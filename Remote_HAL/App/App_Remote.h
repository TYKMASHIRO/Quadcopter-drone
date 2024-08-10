#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "main.h"
#include "gpio.h"

#define Filter_Num 10 // 滑动窗口的长度

#define READ_KEY_LEFT_X HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin)
/* 读取四个微调按键的电平 */

#define READ_KEY_U HAL_GPIO_ReadPin(KEY_U_GPIO_Port, KEY_U_Pin)
#define READ_KEY_D HAL_GPIO_ReadPin(KEY_D_GPIO_Port, KEY_D_Pin)
#define READ_KEY_L HAL_GPIO_ReadPin(KEY_L_GPIO_Port, KEY_L_Pin)
#define READ_KEY_R HAL_GPIO_ReadPin(KEY_R_GPIO_Port, KEY_R_Pin)

struct _Rc
{
    int16_t THR; // 油门：右摇杆上下
    int16_t YAW; // 偏航：左摇杆左右
    int16_t ROL; // 横滚：右摇杆左右
    int16_t PIT; // 俯仰：左摇杆上下
    /* 预留了6个辅助通道：具体什么用途，自己定义，在这里我们不用 */
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
};

struct _Offset
{
    int16_t THR; // 油门：右摇杆上下
    int16_t YAW; // 偏航：左摇杆左右
    int16_t ROL; // 横滚：右摇杆左右
    int16_t PIT; // 俯仰：左摇杆上下
};

struct _Filter
{
    uint32_t sum;
    uint16_t old[Filter_Num];
};

extern struct _Rc rc;
extern struct _Offset offset;

void App_Remote_Stick_Scan();

void App_Remote_KeyPress();

void App_Remote_RemoteData(uint8_t *remote_send);

#endif
