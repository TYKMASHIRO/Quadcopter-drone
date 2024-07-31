#include "App_Flight.h"
#include "Int_MPU6050.h"
#include "adc.h"

typedef volatile struct
{
    float desired;   // ����ֵ
    float prevError; // �ϴ�ƫ��
    float integ;     // �������ۼ�ֵ
    float kp;        // p����
    float ki;        // i����
    float kd;        // d����
    float measured;  // ʵ�ʲ���ֵ
    float out;       // pid���
} PidObject;
PidObject pidPitch;
PidObject pidRoll;
PidObject pidYaw;
PidObject pidRateX;
PidObject pidRateY;
PidObject pidRateZ;

PidObject *pids[] = {&pidPitch, &pidRoll, &pidYaw, &pidRateX, &pidRateY, &pidRateZ};

_stMPU MPU6050;
_stAngle Angle;
int16_t MPU_Offset[6] = {0};
float bat_val = 4000; // ��ʼֵΪ4000mv
extern uint16_t ADC_Value[5];

sLED LED = {1000, AlwaysOff};
uint16_t led_count = 0;

/* ����ң�����ݵĽṹ�� */
_stRemote remote;

/* ������־λ:0��ʾ������1��ʾ���� */
uint8_t unlock_flag = 0;

void App_Flight_VolCheck(void)
{
    /* ��ƫ�����
        ��ѹֵƫ��= ��ǰ������ѹֵ - ֮ǰ�ĵ�ѹֵ
        ��ѹֵ = 0.2 * (ƫ��ֵ - ֮ǰ��ѹֵ)
        ==���𿪹�ʽ�� ==�� ��ѹֵ = 0.2 * ��ǰֵ + 0.8 * ֮ǰ��ѹֵ
        bat_val += 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2) - bat_val);
        �� ���Ϊһ�׵�ͨ�˲�
        ��ѹֵ = 0.2 * ��ǰ��ѹ����ֵ + 0.8 * ֮ǰ�ĵ�ѹֵ
     */
    bat_val = 0.2 * ((ADC_Value[0] * 3300 / 4095 * 2)) + 0.8 * bat_val;
}

void App_Flight_MPU_Data(void)
{
    /* 1����ȡԭʼ���� */
    Int_MPU6050_GetAccl(&MPU6050.accX, &MPU6050.accY, &MPU6050.accZ);
    Int_MPU6050_GetGyro(&MPU6050.gyroX, &MPU6050.gyroY, &MPU6050.gyroZ);

    printf("============MPU ��ʼֵ ==============\r\n");
    printf("accX=%d\r\n", MPU6050.accX);
    printf("accY=%d\r\n", MPU6050.accY);
    printf("accZ=%d\r\n", MPU6050.accZ);
    printf("gyroX=%d\r\n", MPU6050.gyroX);
    printf("gyroY=%d\r\n", MPU6050.gyroY);
    printf("gyroZ=%d\r\n", MPU6050.gyroZ);

    /* 2: ��ƫУ׼ */
    // MPU6050.accX = MPU6050.accX - MPU_Offset[0];
    // MPU6050.accY = MPU6050.accY - MPU_Offset[1];
    // MPU6050.accZ = MPU6050.accZ - MPU_Offset[2];
    // MPU6050.gyroX = MPU6050.gyroX - MPU_Offset[3];
    // MPU6050.gyroY = MPU6050.gyroY - MPU_Offset[4];
    // MPU6050.gyroZ = MPU6050.gyroZ - MPU_Offset[5];

    // printf("============MPU �˲����ֵ ==============\r\n");
    // printf("accX=%d\r\n", MPU6050.accX);
    // printf("accY=%d\r\n", MPU6050.accY);
    // printf("accZ=%d\r\n", MPU6050.accZ);
    // printf("gyroX=%d\r\n", MPU6050.gyroX);
    // printf("gyroY=%d\r\n", MPU6050.gyroY);
    // printf("gyroZ=%d\r\n", MPU6050.gyroZ);

    /* 3�� �Լ��ٶȽ��м���һά�������˲� */
    /* 3.1 X����ٶ��˲� */
    // Com_Kalman_1(&ekf[0], MPU6050.accX); // ��X����ٶȣ����м���һά�������˲�
    // MPU6050.accX = (int16_t)ekf[0].out;  // ���˲���Ľ������ֵ��accX
    // /* 3.2 Y����ٶ��˲� */
    // Com_Kalman_1(&ekf[1], MPU6050.accY); // ��Y����ٶȣ����м���һά�������˲�
    // MPU6050.accY = (int16_t)ekf[1].out;  // ���˲���Ľ������ֵ��accY
    // /* 3.3 Z����ٶ��˲� */
    // Com_Kalman_1(&ekf[2], MPU6050.accZ); // ��Z����ٶȣ����м���һά�������˲�
    // MPU6050.accZ = (int16_t)ekf[2].out;  // ���˲���Ľ������ֵ��accZ

    // /* 4�� �Խ��ٶȽ��м򵥵�һ�׵�ͨ�˲� */
    // static int16_t lastGyro[3] = {0};
    // /* 4.1 X����ٶȵ�ͨ�˲� */
    // MPU6050.gyroX = 0.85 * lastGyro[0] + 0.15 * MPU6050.gyroX; // 0.85 * ֮ǰ��ֵ + 0.15 * ��ε�ֵ
    // lastGyro[0] = MPU6050.gyroX;
    // /* 4.2 Y����ٶȵ�ͨ�˲� */                                // ���±���Ľ��ٶ�ֵ���´���
    // MPU6050.gyroY = 0.85 * lastGyro[1] + 0.15 * MPU6050.gyroY; // 0.85 * ֮ǰ��ֵ + 0.15 * ��ε�ֵ
    // lastGyro[1] = MPU6050.gyroY;                               // ���±���Ľ��ٶ�ֵ���´���
    // /* 4.3 Z����ٶȵ�ͨ�˲� */
    // MPU6050.gyroZ = 0.85 * lastGyro[2] + 0.15 * MPU6050.gyroZ; // 0.85 * ֮ǰ��ֵ + 0.15 * ��ε�ֵ
    // lastGyro[2] = MPU6050.gyroZ;                               // ���±���Ľ��ٶ�ֵ���´���

    // printf("============MPU �˲����ֵ ==============\r\n");
    // printf("accX=%d\r\n", MPU6050.accX);
    // printf("accY=%d\r\n", MPU6050.accY);
    // printf("accZ=%d\r\n", MPU6050.accZ);
    // printf("gyroX=%d\r\n", MPU6050.gyroX);
    // printf("gyroY=%d\r\n", MPU6050.gyroY);
    // printf("gyroZ=%d\r\n", MPU6050.gyroZ);
}