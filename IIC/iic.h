/**
  ******************************************************************************
  * @file    iic.h
  * @author  yiyah (yiyah@github.com)
  * @brief   Header file of IIC BSP module.
  * @version 0.1
  * @date    2024-01-25
  * 
  * @copyright Copyright (c) 2024
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IIC_H
#define __IIC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f103xb.h"        // for GPIO_TypeDef

/** @addtogroup BSP
  * @{
  */

/** @addtogroup IIC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/** @defgroup IIC_Exported_types IIC Exported types
  * @{
  */

/**
 * @brief IIC Types Definition
 * 
 * @note  ths struct only used in local
 */
typedef struct IIC_BUS_s
{
    u8 num_of_devices;
    GPIO_TypeDef *port_of_clk;
    u16 pin_of_clk;
    GPIO_TypeDef *port_of_data;
    u16 pin_of_data;

    u8 (*write)(u8 *devAddr, u8 reg, u8 len, u8 *data);
    u8 (*read)(u8 *devAddr, u8 reg, u8 len, u8 *data);
} IIC_BUS_t;

/**
 * @brief IIC Devices Definition
 * @note  this struct can use in global
 */
typedef struct IIC_device_s
{
    u8 dev_addr;
    IIC_BUS_t *iic_bus;
} IIC_device_t;
/**
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/** @addtogroup IIC_Exported_variables
  * @{
  */
extern IIC_BUS_t iic_swBus;
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @addtogroup IIC_Exported_functions
  * @{
  */
void BSP_IIC_Init();
/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @}
  */
/**
  * @}
  */
#endif
