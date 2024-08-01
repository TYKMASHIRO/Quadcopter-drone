#include "App_Flight.h"

PidObject pidPitch;
PidObject pidRoll;
PidObject pidYaw;
PidObject pidRateX;
PidObject pidRateY;
PidObject pidRateZ;

PidObject *pids[] = {&pidPitch, &pidRoll, &pidYaw, &pidRateX, &pidRateY, &pidRateZ};

extern const float Gyro_G;

/* 4个电机的pwm */
int16_t motor1 = 0; // 右后
int16_t motor2 = 0; // 右前
int16_t motor3 = 0; // 左前
int16_t motor4 = 0; // 左后

_stMPU MPU6050;
_stAngle Angle;
int16_t MPU_Offset[6] = {0};
float bat_val = 4000; // 初始值为4000mv

extern uint16_t ADC_Value[5];

sLED LED = {1000, AlwaysOff};
uint16_t led_count = 0;

/* 接收遥控数据的结构体 */
_stRemote remote;

/* 解锁标志位:0表示锁定，1表示解锁 */
uint8_t unlock_flag = 0;

void App_Flight_VolCheck(void)
{
    /* 对偏差积分
       电压值偏差= 当前采样电压值 - 之前的电压值
       电压值 = 0.2 * (偏差值 - 之前电压值)
       ==》拆开公式后 ==》 电压值 = 0.2 * 当前值 + 0.8 * 之前电压值
       bat_val += 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2) - bat_val);
       或 理解为一阶低通滤波
       电压值 = 0.2 * 当前电压采样值 + 0.8 * 之前的电压值
    */
    bat_val = 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2)) + 0.8 * bat_val;
}

/**
 * @description: 获取MPU六轴原始数据，进行零偏校准滤波
 * @return {*}
 */
void App_Flight_MPU_Data(void)
{
    /* 1、获取原始数据 */
    Int_MPU6050_GetAccl(&MPU6050.accX, &MPU6050.accY, &MPU6050.accZ);
    Int_MPU6050_GetGyro(&MPU6050.gyroX, &MPU6050.gyroY, &MPU6050.gyroZ);

    // printf("============MPU Initial Value ==============\r\n");
    // printf("accX=%d\r\n", MPU6050.accX);
    // printf("accY=%d\r\n", MPU6050.accY);
    // printf("accZ=%d\r\n", MPU6050.accZ);
    // printf("gyroX=%d\r\n", MPU6050.gyroX);
    // printf("gyroY=%d\r\n", MPU6050.gyroY);
    // printf("gyroZ=%d\r\n", MPU6050.gyroZ);

    // /* 2: 零偏校准 */
    // MPU6050.accX = MPU6050.accX - MPU_Offset[0];
    // MPU6050.accY = MPU6050.accY - MPU_Offset[1];
    // MPU6050.accZ = MPU6050.accZ - MPU_Offset[2];
    // MPU6050.gyroX = MPU6050.gyroX - MPU_Offset[3];
    // MPU6050.gyroY = MPU6050.gyroY - MPU_Offset[4];
    // MPU6050.gyroZ = MPU6050.gyroZ - MPU_Offset[5];

    // printf("============MPU 婊ゆ尝鍚庣殑鍊? ==============\r\n");
    // printf("accX=%d\r\n", MPU6050.accX);
    // printf("accY=%d\r\n", MPU6050.accY);
    // printf("accZ=%d\r\n", MPU6050.accZ);
    // printf("gyroX=%d\r\n", MPU6050.gyroX);
    // printf("gyroY=%d\r\n", MPU6050.gyroY);
    // printf("gyroZ=%d\r\n", MPU6050.gyroZ);

    /* 3、 对加速度进行简易一维卡尔曼滤波 */
    /* 3.1 X轴加速度滤波 */
    Com_Kalman_1(&ekf[0], MPU6050.accX); // 对X轴加速度，进行简易一维卡尔曼滤波
    MPU6050.accX = (int16_t)ekf[0].out;  // 将滤波后的结果，赋值给accX
    /* 3.2 Y轴加速度滤波 */
    Com_Kalman_1(&ekf[1], MPU6050.accY); // 对Y轴加速度，进行简易一维卡尔曼滤波
    MPU6050.accY = (int16_t)ekf[1].out;  // 将滤波后的结果，赋值给accY
    /* 3.3 Z轴加速度滤波 */
    Com_Kalman_1(&ekf[2], MPU6050.accZ); // 对Z轴加速度，进行简易一维卡尔曼滤波
    MPU6050.accZ = (int16_t)ekf[2].out;  // 将滤波后的结果，赋值给accZ

    /* 4、 对角速度进行简单的一阶低通滤波 */
    static int16_t lastGyro[3] = {0};
    /* 4.1 X轴角速度低通滤波 */
    MPU6050.gyroX = 0.85 * lastGyro[0] + 0.15 * MPU6050.gyroX; // 0.85 * 之前的值 + 0.15 * 这次的值
    lastGyro[0] = MPU6050.gyroX;
    /* 4.2 Y轴角速度低通滤波 */                                // 更新保存的角速度值，下次用
    MPU6050.gyroY = 0.85 * lastGyro[1] + 0.15 * MPU6050.gyroY; // 0.85 * 之前的值 + 0.15 * 这次的值
    lastGyro[1] = MPU6050.gyroY;                               // 更新保存的角速度值，下次用
    /* 4.3 Z轴角速度低通滤波 */
    MPU6050.gyroZ = 0.85 * lastGyro[2] + 0.15 * MPU6050.gyroZ; // 0.85 * 之前的值 + 0.15 * 这次的值
    lastGyro[2] = MPU6050.gyroZ;                               // 更新保存的角速度值，下次用

    // printf("============MPU The filtered value ==============\r\n");
    // printf("accX=%d\r\n", MPU6050.accX);
    // printf("accY=%d\r\n", MPU6050.accY);
    // printf("accZ=%d\r\n", MPU6050.accZ);
    // printf("gyroX=%d\r\n", MPU6050.gyroX);
    // printf("gyroY=%d\r\n", MPU6050.gyroY);
    // printf("gyroZ=%d\r\n", MPU6050.gyroZ);
}

void App_Flight_MPU_Offsets()
{
    /* 1、 */
    uint8_t gyro_i = 30;
    int32_t buff[6] = {0};

    /* 判断传感器是否处于静止状态：取 陀螺仪角速度 两次的差值，在一定范围内，认为是静止的 */
    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;
    int16_t LastGyro[3] = {0}; // 保存上一次陀螺仪三个轴的数据
    int16_t Err_Gyro[3] = {0}; // 保存每一次计算出来的三轴陀螺仪偏差值
    HAL_Delay(10);
    /* 更新MPU的值 */
    App_Flight_MPU_Data();
    printf("============MPU Initial Value ==============\r\n");
    printf("accX=%d\r\n", MPU6050.accX);
    printf("accY=%d\r\n", MPU6050.accY);
    printf("accZ=%d\r\n", MPU6050.accZ);
    printf("gyroX=%d\r\n", MPU6050.gyroX);
    printf("gyroY=%d\r\n", MPU6050.gyroY);
    printf("gyroZ=%d\r\n", MPU6050.gyroZ);
    /* 连续判断30次满足静止条件，才往后执行 */
    while (gyro_i--)
    {
        /* 内层循环，每次要等待满足静止条件为止 */
        do
        {

            /* 计算偏差值 */
            Err_Gyro[0] = MPU6050.gyroX - LastGyro[0];
            Err_Gyro[1] = MPU6050.gyroY - LastGyro[1];
            Err_Gyro[2] = MPU6050.gyroZ - LastGyro[2];
            /* 保存当前值，下次用 */
            LastGyro[0] = MPU6050.gyroX;
            LastGyro[1] = MPU6050.gyroY;
            LastGyro[2] = MPU6050.gyroZ;
        } while (
            /* 每个轴的2次角速度差值，必须在偏差范围内，才认为满足条件===》跳出内层循环 */
            (Err_Gyro[0] < MIN_GYRO_QUIET || Err_Gyro[0] > MAX_GYRO_QUIET || Err_Gyro[1] < MIN_GYRO_QUIET || Err_Gyro[1]) > (MAX_GYRO_QUIET || Err_Gyro[2] < MIN_GYRO_QUIET || Err_Gyro[2] > MAX_GYRO_QUIET)

        );
    }

    /* 取多次原始值，取平均值 : 去头100次，100+256（2的8次方）=356 */
    for (uint16_t i = 0; i < 356; i++)
    {
        /* 更新MPU的值 */
        HAL_Delay(10);
        App_Flight_MPU_Data();
        if (i >= 100)
        {

            /* 累加后面256次的值 */
            buff[0] += MPU6050.accX;
            buff[1] += MPU6050.accY;
            buff[2] += MPU6050.accZ - 16384;
            buff[3] += MPU6050.gyroX;
            buff[4] += MPU6050.gyroY;
            buff[5] += MPU6050.gyroZ;
        }
    }

    /* 除以256次，直接右移8位 */
    for (uint8_t i = 0; i < 6; i++)
    {
        MPU_Offset[i] = buff[i] >> 8; // shift right
    }
    printf("============MPU The filtered value ==============\r\n");
    printf("accX=%d\r\n", MPU6050.accX);
    printf("accY=%d\r\n", MPU6050.accY);
    printf("accZ=%d\r\n", MPU6050.accZ);
    printf("gyroX=%d\r\n", MPU6050.gyroX);
    printf("gyroY=%d\r\n", MPU6050.gyroY);
    printf("gyroZ=%d\r\n", MPU6050.gyroZ);
}
