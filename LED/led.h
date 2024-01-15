/**
 * @file    led.h
 * @author  yiyah (yiyah@github.com)
 * @brief   Header file of LED BSP module.
 * @version 0.1
 * @date    2024-01-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H__
#define __LED_H__

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup LED
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/** @defgroup LED_Exported_types LED Exported types
  * @{
  */

/**
 * @brief LED Types Definition
 * @note  The enumeration value of @ref LED_TOTAL should be calculated automatically.
 *        So do not change its position
 */
typedef enum
{
    /* ADD LED BEGIN 1 */
    LED_BLUE = 0,               /*!< index of LED_BLUE */

    /* ADD LED END 1 */
    LED_TOTAL                   /*!< total number of LEDs */
} LED_Type;

/**
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup LED_Exported_Functions
  * @{
  */
void BSP_LED_ON(LED_Type led_index);
void BSP_LED_OFF(LED_Type led_index);
void BSP_LED_Toggle(LED_Type led_index);

/**
  * @}
  */
/**
  * @}
  */
/**
  * @}
  */
#endif /* __LED_H__ */

