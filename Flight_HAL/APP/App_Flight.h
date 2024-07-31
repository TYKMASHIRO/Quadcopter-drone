#ifndef __APP_FLIGHT_H
#define __APP_FLIGHT_H

#include "main.h"
#include "LED.h"
#include "Int_MPU6050.h"

#include "tim.h"

/* ���弸���꣬����������Ĳ�ͬ״̬ */
#define ENMERGENCY_0 0 // ����׶��ж������Ƿ���ͣ�����ǣ���ɵڶ��׶�
#define WAITING_1 1    // ����׶��ж������Ƿ���ͣ�����ǣ���ɵڶ��׶�
#define WAITING_2 2    // ����׶��ж������Ƿ���ߣ�����ǣ���ɵ����׶�
#define WAITING_3 3    // ����׶��ж������Ƿ���ͣ�����ǣ���ɵ��Ľ׶�
#define WAITING_4 4    // ����׶α�ʾ�������ɹ��׶�
#define PROCESS 5      // ����׶α�ʾ����ʽ���ƽ׶�
#define EXIT 6         // ����׶α�ʾ����ʽ���ƽ׶�

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
    float pitch; /* ŷ�����е� ������ */
    float roll;  /* ŷ�����е� ����� */
    float yaw;   /* ŷ�����е� ƫ���� */
} _stAngle;

typedef struct
{
    int16_t THR; // ���ţ���ҡ������
    int16_t YAW; // ƫ������ҡ������
    int16_t ROL; // �������ҡ������
    int16_t PIT; // ��������ҡ������
    /* Ԥ����6������ͨ��������ʲô��;���Լ����壬���������ǲ��� */
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
} _stRemote;

/* ��ͷ�ļ���extern�������������c�ж���ı��� */
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
