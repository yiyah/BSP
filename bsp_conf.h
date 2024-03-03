#ifndef __BSP_CONF_H
#define __BSP_CONF_H

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the BSP driver
  */

#define BSP_IIC_MODULE_ENABLED
#define BSP_LED_MODULE_ENABLED
#define BSP_DEBUG_MODULE_ENABLED
#define BSP_MPU6050_MODULE_ENABLED
#define BSP_MOTOR_MODULE_ENABLED

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */
#ifdef BSP_LED_MODULE_ENABLED
#include "led.h"
#endif /* BSP_LED_MODULE_ENABLED */

#ifdef BSP_IIC_MODULE_ENABLED
#include "iic.h"
#endif /* BSP_IIC_MODULE_ENABLED */

#ifdef BSP_DEBUG_MODULE_ENABLED
#include "log.h"
#endif /* BSP_DEBUG_MODULE_ENABLED */

#ifdef BSP_MPU6050_MODULE_ENABLED
#include "mpu6050.h"
#endif /* BSP_MPU6050_MODULE_ENABLED */

#ifdef BSP_MOTOR_MODULE_ENABLED
#include "motor.h"
#endif /* BSP_MOTOR_MODULE_ENABLED */

#endif /* __BSP_CONF_H */
