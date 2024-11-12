/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include "types.h"
#include "led.h"
#include "key.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup KEY KEY
  * @brief KEY BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define delay_ms(xms)     do { \
                            u16 _i, _j; \
                            for (_i = 0;_i < (xms); _i++){ \
                                for (_j = 0;_j < (1000); _j++){ \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                    __NOP();__NOP(); \
                                } \
                            }\
                        } while(0)
/* Private variables ---------------------------------------------------------*/
cGPIO LED;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void cGPIO::write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state);
}

void gpio_write(GPIO_PinState state)
{
    LED.write(state);
}

void HAL_GPIO_EXTI_Callback(u16 GPIO_Pin)
{
    if (KEY1_Pin == GPIO_Pin)
    {
        delay_ms(10);
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0)
        {
            BSP_LED_Toggle(LED_BLUE);
        }
    }
    else
    {
        
    }
}

/**
  * @}
  */

/**
  * @}
  */
