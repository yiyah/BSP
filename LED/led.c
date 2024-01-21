/**
 * @file    led.c
 * @author  yiyah (yiyah@github.com)
 * @brief   LED BSP module driver.
 * @version 0.1
 * @date    2024-01-14
 * 
 * @copyright Copyright (c) 2024
 * 
 @verbatim
 ==============================================================================
                        ##### How to use this driver #####
 ==============================================================================
 [..]
   (#) step1: Define the GPIO info of LED and its work state in LEDs[].

   (#) step2: Define the LED index in enum LED_Type.
              It must be in the same order as defined in LEDs[].
 @endverbatim
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"
#include "types.h"
#include "led.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup LED LED
  * @brief LED BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup LED_Private_variables LED Private variables
  * @{
  */

/**
 * @brief LED structure definition
 */
typedef struct
{
    GPIO_TypeDef    *GPIOx;
    u16             GPIO_Pin;
    GPIO_PinState   work;           /*!< the pin is high or low when LED is on */
} LED_TypeDef;

const LED_TypeDef LEDs[] = {
    {GPIOA, GPIO_PIN_4, GPIO_PIN_SET}   /*!< LED_BLUE */
};

/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup LED_Exported_Functions LED Exported Functions
  * @{
  */

/**
 * @brief  Turns selected LED on.
 * @param  led_index: Specifies the Led to be set on.
 *         This parameter can be any value of @ref LED_Type:
 *           @arg LED_BLUE
 * @retval None
 */
void BSP_LED_ON(LED_Type led_index)
{
    HAL_GPIO_WritePin(LEDs[led_index].GPIOx, LEDs[led_index].GPIO_Pin, LEDs[led_index].work);
}

/**
 * @brief  Turns selected LED off.
 * @param  led_index: Specifies the Led to be set off.
 *         This parameter can be any value of @ref LED_Type:
 *           @arg LED_BLUE
 * @retval None
 */
void BSP_LED_OFF(LED_Type led_index)
{
    HAL_GPIO_WritePin(LEDs[led_index].GPIOx, LEDs[led_index].GPIO_Pin, !LEDs[led_index].work);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *         This parameter can be any value of @ref LED_Type:
  *           @arg LED_BLUE
  * @retval None
  */
void BSP_LED_Toggle(LED_Type led_index)
{
  HAL_GPIO_TogglePin(LEDs[led_index].GPIOx, LEDs[led_index].GPIO_Pin);
}

/**
  * @}
  */
/**
  * @}
  */
/**
  * @}
  */
