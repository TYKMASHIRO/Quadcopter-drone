#ifndef __APP_FLIGHT_H
#define __APP_FLIGHT_H

#include "main.h"
#include "LED.h"
#include "Int_MPU6050.h"
#include "Com_Kalman.h"
#include "Com_PID.h"
#include "tim.h"
#include "NRF24L01.h"
#include "stdio.h"
/* 定义几个宏，来代表解锁的不同状态 */
#define ENMERGENCY_0 0 // 这个阶段判断油门是否最低，如果是，变成第二阶段
#define WAITING_1 1    // 这个阶段判断油门是否最低，如果是，变成第二阶段
#define WAITING_2 2    // 这个阶段判断油门是否最高，如果是，变成第三阶段
#define WAITING_3 3    // 这个阶段判断油门是否最低，如果是，变成第四阶段
#define WAITING_4 4    // 这个阶段表示，解锁成功阶段
#define PROCESS 5      // 这个阶段表示，正式控制阶段
#define EXIT 6         // 这个阶段表示，正式控制阶段

#define LIMIT(x, min, max) (x < min) ? min : ((x > max) ? max : x)

typedef struct
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} _stMPU;

typedef struct
{
    float pitch; /* 欧拉角中的 俯仰角 */
    float roll;  /* 欧拉角中的 横滚角 */
    float yaw;   /* 欧拉角中的 偏航角 */
} _stAngle;

typedef struct
{
    int16_t THR; // 油门：左摇杆上下
    int16_t YAW; // 偏航：左摇杆左右
    int16_t ROL; // 横滚：右摇杆左右
    int16_t PIT; // 俯仰：右摇杆上下
    /* 预留了6个辅助通道：具体什么用途，自己定义，在这里我们不用 */
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
} _stRemote;

/* 在头文件中extern，方便别人引入c中定义的变量 */
extern _stMPU MPU6050;
extern _stAngle Angle;

extern sLED LED;

extern _stRemote remote;

void App_Flight_VolCheck(void);

void App_Flight_MPU_Data(void);

void App_Flight_MPU_Offsets();

void App_Flight_Remote_Check(uint8_t *buf, uint8_t len);

void App_Flight_RC_Unlock();

void App_Flight_RC_Analysis();

void App_Flight_PID_Control(float dt);

void App_Flight_Motor_Control();

void App_Flight_Mode_Control(void);

void App_PID_Param_Init();

#endif
