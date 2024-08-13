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
            App_Flight_MPU_Data();
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

    printf("============MPU The filtered value ==============\r\n");
    printf("accX=%d\r\n", MPU6050.accX);
    printf("accY=%d\r\n", MPU6050.accY);
    printf("accZ=%d\r\n", MPU6050.accZ);
    printf("gyroX=%d\r\n", MPU6050.gyroX);
    printf("gyroY=%d\r\n", MPU6050.gyroY);
    printf("gyroZ=%d\r\n", MPU6050.gyroZ);
    for (uint8_t i = 0; i < 6; i++)
    {
        MPU_Offset[i] = buff[i] >> 8; // shift right
    }
}
/**
 * @description: 解析校验接收到的遥控数据
 * @param {uint8_t} *buf 接收到的遥控数据
 * @param {uint8_t} len 数组长度
 * @return {*}
 */
void App_Flight_Remote_Check(uint8_t *buf, uint8_t len)
{
    connect_flag++;
    /* ！接收函数执行完之后，执行该函数，用来判断是否失联不然在2.4G信号外就不可控，如果正常，flag++后一定=1，除了1都不正常 */
    if (connect_flag == 1)
    {

        uint32_t rc_sum = 0;
        uint32_t flight_sum = 0;
        /* 1 帧头校验 */
        if (!((*buf == 0xAA) && (*(buf + 1) == 0xAF)))
        {
            /* 不满足帧头，直接返回 */
            printf("head error,head0=%02x,head1=%02x\r\n", *buf, *(buf + 1));
            return;
        }

        /* 2 校验和 校验 */
        /* 2.1 取出校验和 */
        rc_sum = *(buf + len - 4) << 24 | *(buf + len - 3) << 16 | *(buf + len - 2) << 8 | *(buf + len - 1);
        /* 2.2 通过接收数据，自己计算一下校验和 */
        for (uint8_t i = 0; i < len - 4; i++)
        {
            flight_sum += buf[i];
        }

        /* 2.3 两个做比较 */
        if (flight_sum != rc_sum)
        {
            /* 计算的校验和 不等于 本身的校验和 ，非法数据，直接丢弃 */
            printf("check sum error.....f_sum=%04x,rc_sum=%04x\r\n", flight_sum, rc_sum);
            return;
        }

        /* 3 判断功能字，进行相应的处理：这里只有遥控数据功能，03 */
        if (buf[2] == 0x03)
        {
            /* 取出遥控的数据:从索引4开始，是遥控数据了 */
            remote.THR = (int16_t)(*(buf + 4) << 8 | *(buf + 5));
            remote.YAW = (int16_t)(*(buf + 6) << 8 | *(buf + 7));
            remote.ROL = (int16_t)(*(buf + 8) << 8 | *(buf + 9));
            remote.PIT = (int16_t)(*(buf + 10) << 8 | *(buf + 11));
            /* 辅助通道的数据aux1-aux6:从索引12开始 */
            remote.AUX1 = (int16_t)(*(buf + 12) << 8 | *(buf + 13));
            remote.AUX2 = (int16_t)(*(buf + 14) << 8 | *(buf + 15));
            remote.AUX3 = (int16_t)(*(buf + 16) << 8 | *(buf + 17));
            remote.AUX4 = (int16_t)(*(buf + 18) << 8 | *(buf + 19));
            remote.AUX5 = (int16_t)(*(buf + 20) << 8 | *(buf + 21));
            remote.AUX6 = (int16_t)(*(buf + 22) << 8 | *(buf + 23));

            /* ====================测试：打印解析完的数据===================== */
            printf("THR=%d\r\n", remote.THR);
            printf("YAW=%d\r\n", remote.YAW);
            printf("ROL=%d\r\n", remote.ROL);
            printf("PIT=%d\r\n", remote.PIT);
        }
    }
    if (connect_flag > 5000)
    {
        /* ！长时间失联，flag会加到一个很大的数，为了避免越界，到一定数，重新置为1（不能是0，因为还是失联） */
        /* 这里越界的条件和复位值，注意参考后面 失联处理函数的判断时间 */
        connect_flag = 1251;
    }
}
/**
 * @description: 根据指令，判断和解锁飞机
 * @return {*}
 */
void App_Flight_RC_Unlock()
{
    /* 解锁指令：油门最低 --》 油门最高 --》 油门最低 ==》 解锁 */
    static uint8_t status = WAITING_1; // 默认阶段1
    static uint16_t time_count = 0;

    if (status == ENMERGENCY_0)
    {
        /* 为了安全，加一道判断，如果是紧急状态，直接退出 */
        status = EXIT;
    }

    switch (status)
    {
    case WAITING_1:
        /* 阶段1：判断油门是否最低 */
        if (remote.THR < 1030)
        {
            printf(">>>>>>>>>>>>>>> w1\r\n");
            /* 1030放点水，不要卡太死。油门是低，准备进入阶段2 */
            status = WAITING_2;
        }
        break;
    case WAITING_2:
        /* 阶段2：说明之前已经油门最低，现在要判断油门是否最高 */
        if (remote.THR > 1900)
        {
            printf("<<<<<<<<<<<< w2\r\n");
            /* 1900放点水，不卡太死。油门是高，准备进入阶段3 */
            time_count++;
            if (time_count > 50) // 假设该函数10ms调用一次，50*10ms=500ms
            {
                status = WAITING_3;
                time_count = 0;
            }
        }
        break;
    case WAITING_3:
        /* 阶段3：标志前面已经 油门最低-》油门最高（保持时间）。现在判断油门是否拉低，如果是，准别进入阶段4 */
        if (remote.THR < 1030)
        {
            printf(">>>>>>>>>>>>>>> w3\r\n");
            /* 1030放水。油门是低，准备进入阶段4 */
            status = WAITING_4;
        }
        break;
    case WAITING_4:
        /* 阶段4：前面解锁步骤完成。进行一些解锁后的处理，比如标志位 */
        /* 将解锁标志位置位1，表示解锁完成状态，方便其他函数做判断 */
        printf("<<<<<<<<<<<<<<<<<< w4\r\n");
        unlock_flag = 1;
        status = PROCESS;

        break;
    case PROCESS:
        /* 说明解锁完毕，准备阶段渡过，正式进入控制 */
        /* 为了安全，解锁后长时间不操作，又锁定 */
        printf("======================== p\r\n");
        if (remote.THR < 1030)
        {
            /* 说明油门没有动，开始计时 */
            if (time_count++ > 1000)
            {
                /* 说明已经长时间没操作=》清标志位、回复到阶段1、count清零 */
                unlock_flag = 0;
                status = WAITING_1;
                time_count = 0;
            }
        }
        /* 如果标志位被清零，直接退出控制，进入退出状态 */
        if (!unlock_flag)
        {
            status = EXIT;
        }
        break;
    case EXIT:
        // 这里直接进行退出
        printf("!!!!!!!!!!!!!!!!! exit\r\n");
        unlock_flag = 0;
        time_count = 0;
        status = WAITING_1;
        break;
    default:
        status = EXIT;
    }
}

/**
 * @description: 遥控指令解析（比如解锁指令），添加失联后的处理逻辑
 * @return {*}
 */
void App_Flight_RC_Analysis()
{
    static uint16_t flag = 4;
    static uint16_t thr_count = 0;
    static uint16_t disconnect_count = 0;

    if (connect_flag == 1)
    {
        /* 执行解锁指令 */
        App_Flight_RC_Unlock();
        while (flag--)
        {
            printf("iscoming\r\n");
        }
        flag = 4;
    }
    else
    {
        /* 失联: 超过3s，判定失联，进行处理 */
        if (connect_flag > 1250) // 测试时，稍微调大点观察
        {
            printf("remote disconnect......\r\n");
            /* 2 慢慢降低油门落地 */
            /* 2.1 除了油门，全部归中 */
            remote.PIT = 1500;
            remote.ROL = 1500;
            remote.YAW = 1500;
            /* 2.2 油门很低，直接清 */
            if (remote.THR < 1200)
            {
                remote.THR = 1000;
                /* 进入这个if，有两种情况：
                    1、原来就低于1200
                    2、原来油门不低，但是缓慢减小，最终捡到低于1200，进入这个if */
                printf("restart 2.4G\r\n");
                while (NRF24L01_Check())
                    ;
                // 进行重新连接
                NRF24L01_RX_Mode();
                printf("restart 2.4G suc\r\n");
            }
            /* 2.3 原先油门不算低，缓慢减少油门值 */
            else
            {
                if (thr_count++ > 100) // 假设10ms执行一次，100次=1s
                {
                    printf("thr -50\r\n");
                    remote.THR -= 50;
                    thr_count = 0;
                }
            }
            remote.THR = LIMIT(remote.THR, 1000, 2000);
        }
    }
}
void App_Flight_PID_Control(float dt)
{
    static uint8_t status = WAITING_1;

    switch (status)
    {
    case WAITING_1: /* 1、阶段一：根据标志位 判断是否解锁，则进入阶段2 */
        /* code */
        if (unlock_flag == 1)
        {
            status = WAITING_2;
        }
        break;
    case WAITING_2: /* 2、阶段二：复位PID，进入正式阶段 */
        /* code */
        ResetPID(pids, 6);
        status = PROCESS;
        break;
    case PROCESS: /* 3、正式阶段： PID计算 */
        /* 赋值角度的测量值 */
        pidPitch.measured = Angle.pitch;
        pidRoll.measured = Angle.roll;
        pidYaw.measured = Angle.yaw;
        /* 赋值角速度的测量值 */
        pidRateX.measured = MPU6050.gyroX * Gyro_G;
        pidRateY.measured = MPU6050.gyroY * Gyro_G;
        pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
        /*
            俯仰角 ---》 Y轴角速度
            横滚角 ---》 X轴角速度
            偏航角 ---》 Z轴角速度
         */
        CasecadePID(&pidPitch, &pidRateY, dt);
        CasecadePID(&pidRoll, &pidRateX, dt);
        CasecadePID(&pidYaw, &pidRateZ, dt);

        break;

    default:
        break;
    }
}

/**
 * @description: 电机控制
 * @return {*}
 */
void App_Flight_Motor_Control()
{
    // App_Flight_RC_Unlock();
    static uint8_t status = WAITING_1;

    switch (status)
    {
    case WAITING_1: /* 阶段一：pwm=0，判断解锁标志位 */
        /* 为了安全，初始强制为0 */
        motor1 = motor2 = motor3 = motor4 = 0;
        if (unlock_flag == 1)
        {
            status = WAITING_2;
        }
        break;
    case WAITING_2: /* 阶段二：判断油门动了 > 1100,进入正式控制 */
        if (remote.THR > 1100)
        {
            status = PROCESS;
        }

        break;
    case PROCESS: /* 正式控制: 摇杆-1000，如果油门<1050 pwm=0 ，限制到900， + 3个PID值 */
        int16_t thr_temp;
        /* 1、 油门值转成pwm的范围值 油门值 - 1000 */
        thr_temp = remote.THR - 1000;
        /* 2、 安全考虑，如果油门低，直接不转了 */
        if (remote.THR < 1020)
        {
            motor1 = motor2 = motor3 = motor4 = 0;
            break;
        }
        /* 3、 预留100给PID控制 */
        motor1 = motor2 = motor3 = motor4 = LIMIT(thr_temp, 0, 900);

        /* 4、加上3个PID的值 */
        motor1 += +pidRateX.out + pidRateY.out + pidRateZ.out; // 右后
        motor2 += +pidRateX.out - pidRateY.out - pidRateZ.out; // 右前
        motor3 += -pidRateX.out - pidRateY.out + pidRateZ.out; // 左前
        motor4 += -pidRateX.out + pidRateY.out - pidRateZ.out; // 左后
        break;

    default:
        break;
    }

    printf("motor1=%d,motor2=%d,motor3=%d,motor4=%d\r\n",motor1,motor2,motor3,motor4);
    /* 设置pwm */
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, LIMIT(motor3, 0, 1000)); // 左前
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, LIMIT(motor2, 0, 1000)); // 右前
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, LIMIT(motor4, 0, 1000)); // 左后
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, LIMIT(motor1, 0, 1000)); // 右后
}

/**
 * @description: 摇杆控制飞机前进后退、左右移动
 * @return {*}
 */
void App_Flight_Mode_Control(void)
{
    const float roll_pitch_ratio = 0.04f;
    /* PID计算会让飞机保持平衡，要让飞机移动需要让角度环的期望值非零 */
    pidPitch.desired = -(remote.PIT - 1500) * roll_pitch_ratio; // 摇杆控制
    pidRoll.desired = -(remote.ROL - 1500) * roll_pitch_ratio;  // 摇杆控制
    Angle.yaw = pidYaw.desired = pidYaw.measured = 0;           // 锁定偏航角
}

/**
 * @description: 初始化PID系数
 * @return {*}
 */
void App_PID_Param_Init()
{
    /* 内环 */
    /*
        俯仰角： 内环 Y轴角速度
        横滚角： 内环 X轴角速度
        偏航角： 内环 Z轴角速度
     */
    pidRateX.kp = -3.0f; // -3.0
    pidRateY.kp = 2.0f;  // 2.0
    pidRateZ.kp = -2.0f; // -2.0

    pidRateX.ki = 0.0f;
    pidRateY.ki = 0.0f;
    pidRateZ.ki = 0.0f;

    pidRateX.kd = -0.08f; //-0.08
    pidRateY.kd = 0.08f;  // 0.08
    pidRateZ.kd = 0.00f;

    /* 外环 */
    pidPitch.kp = 7.0f; // 7.0
    pidRoll.kp = 7.0f;  //
    pidYaw.kp = -2.0f;

    pidPitch.ki = 0.0f;
    pidRoll.ki = 0.0f;
    pidYaw.ki = 0.0f;

    pidPitch.kd = 0.0f;
    pidRoll.kd = 0.0f;
    pidYaw.kd = 0.0f;
}
