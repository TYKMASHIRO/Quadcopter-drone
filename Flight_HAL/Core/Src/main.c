/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "LED.h"
#include "Int_MPU6050.h"
#include "App_Flight.h"
#include "Com_IMU.h"
#include "NRF24L01.h"
#include "App_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t pData[1024];
uint16_t ADC_Value[10];
extern uint16_t MPU_Offset[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  Int_MPU6050_Init(); /*mpu6050初始�?*/

  // 启动pwm,当我们用寄存器初始化他不用开,而hal库却要开
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /**
   *  左前：HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   *        右前：HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   *        左后：HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
   *        右后：HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   */
  /**
   * 测试pwm，占空比50%
   */
  /**
   * 为什么这样做：
     访问方式的变化：将 ADC_Value 从 16 位转换为 32 位指针，可能是为了以 32 位宽度读取或写入数据。由于 uint16_t 是 16 位的，而 uint32_t 是 32 位的，转换后可能会导致指针的解引用方式不同，从而影响程序如何访问数组中的数据。

     内存效率：在某些场景中，通过 uint32_t * 访问 uint16_t 数组可能能提高效率，因为有些处理器能够更快地处理 32 位数据。
   * Why do this:
      Access changes: Converting ADC_Value from 16-bit to 32-bit pointer may be done to read or write data with a 32-bit width. Since uint16_t is 16-bit and uint32_t is 32-bit, the conversion may cause pointers to be dereferenced differently, which may affect how the program accesses data in the array.

      Memory efficiency: In some scenarios, accessing uint16_t arrays through uint32_t * may be more efficient because some processors can process 32-bit data faster.
  */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 10);
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 499); // 右后
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 499); // 右前
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 499); // 左前
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 499); // 左后
  // HAL_Delay(1000000);
  /**
   *  test ADC
   *
   */

  while (NRF24L01_Check())
  {
    printf("NRF24L01 Check Failed\r\n");
  }
  printf("remote check ok\r\n");
  // 初始化为发送模式
  NRF24L01_RX_Mode();
  /*初始化内外环的pid参数*/
  App_PID_Param_Init();

  /*进入执行FreeRTOS调度*/
  FreeRTOS_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t rx_buff[28] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // LED_Toggle(GPIOB, GPIO_PIN_1);
    // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
    // {
    //   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 499); // 右后
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 499); // 右前
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 499); // 左前
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 499); // 左后
    // }
    /**
     * @brief Construct a new App_Flight_MPU_Offsets object
     * @test test mpu6050 is ok?
     */
    // App_Flight_MPU_Offsets();
    // GetAngle(&MPU6050, &Angle, 0.006f);
    // printf("pitch=%.1f\r\n", Angle.pitch);
    // printf("roll=%.1f\r\n", Angle.roll);
    // printf("yaw=%.1f\r\n", Angle.yaw);

    /**
     * @brief Construct a new while object
     *        自检2.4G
     */
    NRF24L01_RxPacket(RX_BUFF);

    App_Flight_Remote_Check(RX_BUFF, 28);
    // App_Flight_RC_Unlock();
    HAL_Delay(120);
    // uint8_t rx_buff[28] = {0};
    // // while (NRF24L01_Check())
    // // {
    // //   printf("NRF24L01 is ready\r\n");
    // //   HAL_Delay(1000);
    // // }
    // // printf("NRF24L01 is ok!\r\n");
    // NRF24L01_RxPacket(rx_buff);
    // App_Flight_Remote_Check(rx_buff, 28);
    // App_Flight_RC_Analysis();
    // printf("NRF24L01 is ok! 2222\r\n");
    // HAL_Delay(4);
    // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 499); // 右后
    // if (MPU6050.accX)
    // {
    //   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 499); // 右后
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 499); // 右前
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 499); // 左前
    //   // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 499); // 左后
    // }

    /* USER CODE END 3 */
  }
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
