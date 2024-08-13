#include "App_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "App_Flight.h"
#include "NRF24L01.h"
// #include "Com_IMU.h"

#define START_STACK_DEPTH 256
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void Start_Task(void *pvParameters);

// /* 2ms�������� */
// #define TASK_2MS_STACK_DEPTH 256
// #define TASK_2MS_PRIORITY 2
// TaskHandle_t task_2ms_handle;
// void App_Task_2MS(void *pvParameters);

/* 2ms�������� */
#define TASK_2MS_STACK_DEPTH 256
#define TASK_2MS_PRIORITY 3
TaskHandle_t task_2ms_handle;
void App_Task_2MS(void *pvParameters);

/* 8ms�������� */
#define TASK_8ms_STACK_DEPTH 256
#define TASK_8ms_PRIORITY 2
TaskHandle_t task_8ms_handle;
void App_Task_8MS(void *pvParameters);

/**
 * @description: ��ں���������������������������
 * @return {*}
 */
void FreeRTOS_Start(void)
{
    /* 1. ������������ */
    xTaskCreate((TaskFunction_t)Start_Task,
                (char *)"Start_Task",
                (configSTACK_DEPTH_TYPE)START_STACK_DEPTH,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY,
                (TaskHandle_t *)&start_task_handle);

    /* 2. ���������� */
    vTaskStartScheduler();
}

/**
 * @description: �������񣬴�����������
 * @param {void *} pvParameters
 * @return {*}
 */
void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();
    /* ������ʾ���� */
    // xTaskCreate((TaskFunction_t)App_Task_2MS,
    //             (char *)"App_Task_2MS",
    //             (configSTACK_DEPTH_TYPE)TASK_2MS_STACK_DEPTH,
    //             (void *)NULL,
    //             (UBaseType_t)TASK_2MS_PRIORITY,
    //             (TaskHandle_t *)&task_2ms_handle);

    xTaskCreate((TaskFunction_t)App_Task_2MS,
                (char *)"App_Task_2MS",
                (configSTACK_DEPTH_TYPE)TASK_2MS_STACK_DEPTH,
                (void *)NULL,
                (UBaseType_t)TASK_2MS_PRIORITY,
                (TaskHandle_t *)&task_2ms_handle);

    xTaskCreate((TaskFunction_t)App_Task_8MS,
                (char *)"App_Task_8MS",
                (configSTACK_DEPTH_TYPE)TASK_8ms_STACK_DEPTH,
                (void *)NULL,
                (UBaseType_t)TASK_8ms_PRIORITY,
                (TaskHandle_t *)&task_8ms_handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @description: 2ms��������
 * @param {void *} pvParameters
 * @return {*}
 */
// void App_Task_2MS(void *pvParameters)
// {

//     TickType_t pxPreviousWakeTime;
//     pxPreviousWakeTime = xTaskGetTickCount();

//     while (1)
//     {
//         // /* ��ȡMPU 6������ */
//         // App_Flight_MPU_Data();
//         // /* ����ŷ���ǣ�ע��ڶ�������=��������  1ms=0.001 */
//         // GetAngle(&MPU6050, &Angle, 0.002f);
//         // /* ҡ�˿����ƶ� */
//         // App_Flight_Mode_Control();
//         // /* ��������PID���㣬ע�����=�������� 1ms=0.001 */
//         // App_Flight_PID_Control(0.002f);
//         // /* ������� */
//         // App_Flight_Motor_Control();
//         vTaskDelayUntil(&pxPreviousWakeTime, 2);
//     }
// }

/**
 * @description: 2ms��������
 * @param {void *} pvParameters
 * @return {*}
 */
void App_Task_2MS(void *pvParameters)
{

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 2.4G�������ݣ�ע��������� <= �������� */
        // NRF24L01_RxPacket(RX_BUFF);
        // /* ����ң���������ݣ�����У�� */
        // App_Flight_Remote_Check(RX_BUFF, TX_PLOAD_WIDTH);
        // /* ң�� ָ��Ĵ���������ʧ�� */
        // App_Flight_RC_Analysis();
        App_Flight_MPU_Data();
        /* ����ŷ���ǣ�ע��ڶ�������=��������  1ms=0.001 */
        GetAngle(&MPU6050, &Angle, 0.002f);
        /* ҡ�˿����ƶ� */
        App_Flight_Mode_Control();
        /* ��������PID���㣬ע�����=�������� 1ms=0.001 */
        App_Flight_PID_Control(0.002f);
        /* ������� */
        App_Flight_Motor_Control();
        vTaskDelayUntil(&pxPreviousWakeTime, 16);
        // vTaskDelayUntil(&pxPreviousWakeTime, 2);
    }
}

/**
 * @description: 8ms��������
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_8MS(void *pvParameters)
{

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* �޹ؽ�Ҫ�ĵƿ�ϵͳ������������ */
        NRF24L01_RxPacket(RX_BUFF);
        /* ����ң���������ݣ�����У�� */
        App_Flight_Remote_Check(RX_BUFF, TX_PLOAD_WIDTH);
        /* ң�� ָ��Ĵ���������ʧ�� */
        App_Flight_RC_Analysis();
        vTaskDelayUntil(&pxPreviousWakeTime, 8);
    }
}
