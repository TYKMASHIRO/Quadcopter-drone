#include "LED.h"
// #include "stm32f10x.h"
// #include "INIT.h"
                  // default 300ms flash the status
void LED_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void LED_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}
void LED_Toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}