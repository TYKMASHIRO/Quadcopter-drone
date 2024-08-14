#include "App_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "App_Flight.h"
#include "NRF24L01.h"
#include "Com_IMU.h"

#define START_STACK_DEPTH 256
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void Start_Task(void *pvParameters);

// /* 2ms周期任务 */
// #define TASK_2MS_STACK_DEPTH 256
// #define TASK_2MS_PRIORITY 2
// TaskHandle_t task_2ms_handle;
// void App_Task_2MS(void *pvParameters);

/* 2ms周期任务 */
#define TASK_2MS_STACK_DEPTH 256
#define TASK_2MS_PRIORITY 3
TaskHandle_t task_2ms_handle;
void App_Task_2MS(void *pvParameters);

/* 8ms周期任务 */
#define TASK_8ms_STACK_DEPTH 256
#define TASK_8ms_PRIORITY 4
TaskHandle_t task_8ms_handle;
void App_Task_8MS(void *pvParameters);

/**
 * @description: 入口函数：创建启动任务、启动调度器
 * @return {*}
 */
void FreeRTOS_Start(void)
{
    /* 1. 创建启动任务 */
    xTaskCreate((TaskFunction_t)Start_Task,
                (char *)"Start_Task",
                (configSTACK_DEPTH_TYPE)START_STACK_DEPTH,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY,
                (TaskHandle_t *)&start_task_handle);

    /* 2. 启动调度器 */
    vTaskStartScheduler();
}

/**
 * @description: 启动任务，创建其他任务
 * @param {void *} pvParameters
 * @return {*}
 */
void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();
    /* 创建显示任务 */
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
 * @description: 2ms周期任务
 * @param {void *} pvParameters
 * @return {*}
 */
// void App_Task_2MS(void *pvParameters)
// {

//     TickType_t pxPreviousWakeTime;
//     pxPreviousWakeTime = xTaskGetTickCount();

//     while (1)
//     {
//         // /* 获取MPU 6轴数据 */
//         // App_Flight_MPU_Data();
//         // /* 计算欧拉角，注意第二个参数=调用周期  1ms=0.001 */
//         // GetAngle(&MPU6050, &Angle, 0.002f);
//         // /* 摇杆控制移动 */
//         // App_Flight_Mode_Control();
//         // /* 三个串级PID计算，注意参数=调用周期 1ms=0.001 */
//         // App_Flight_PID_Control(0.002f);
//         // /* 电机控制 */
//         // App_Flight_Motor_Control();
//         vTaskDelayUntil(&pxPreviousWakeTime, 2);
//     }
// }

/**
 * @description: 2ms周期任务
 * @param {void *} pvParameters
 * @return {*}
 */
void App_Task_2MS(void *pvParameters)
{

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 2.4G接收数据，注意调用周期 <= 发送周期 */
        // NRF24L01_RxPacket(RX_BUFF);
        // /* 解析遥控器的数据，进行校验 */
        // App_Flight_Remote_Check(RX_BUFF, TX_PLOAD_WIDTH);
        // /* 遥控 指令的处理：解锁、失联 */
        // App_Flight_RC_Analysis();
        // App_Flight_MPU_Data();
        // /* 计算欧拉角，注意第二个参数=调用周期  1ms=0.001 */
        // GetAngle(&MPU6050, &Angle, 0.002f);
        // /* 摇杆控制移动 */
        // App_Flight_Mode_Control();
        // /* 三个串级PID计算，注意参数=调用周期 1ms=0.001 */
        // App_Flight_PID_Control(0.002f);
        // /* 电机控制 */
        // App_Flight_Motor_Control();
        // vTaskDelayUntil(&pxPreviousWakeTime, 2);
        // vTaskDelayUntil(&pxPreviousWakeTime, 2);
    }
}

/**
 * @description: 8ms周期任务
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_8MS(void *pvParameters)
{

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1)
    {
        App_Flight_MPU_Data();
        /* 计算欧拉角，注意第二个参数=调用周期  1ms=0.001 */
        GetAngle(&MPU6050, &Angle, 0.002f);
        /* 摇杆控制移动 */
        App_Flight_Mode_Control();
        /* 三个串级PID计算，注意参数=调用周期 1ms=0.001 */
        App_Flight_PID_Control(0.002f);
        /* 电机控制 */
        App_Flight_Motor_Control();
        // vTaskDelayUntil(&pxPreviousWakeTime, 2);
        /* 无关紧要的灯控系统任务，周期随意 */
        NRF24L01_RxPacket(RX_BUFF);
        /* 解析遥控器的数据，进行校验 */
        App_Flight_Remote_Check(RX_BUFF, TX_PLOAD_WIDTH);
        /* 遥控 指令的处理：解锁、失联 */
        App_Flight_RC_Analysis();
        vTaskDelayUntil(&pxPreviousWakeTime, 8);
    }
}
