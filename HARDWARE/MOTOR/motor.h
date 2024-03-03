/**
 ******************************************************************************
 * @file    motor.h
 * @author  yiyah (yiyah@github.com)
 * @brief   Header file of MOTOR BSP module.
 * @version 0.1
 * @date    2024-03-03
 * 
 * @copyright Copyright (c) 2024
 * 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup MOTOR_Exported_functions
  * @{
  */
void BSP_InitMotor();
void BSP_StopMotor();
void BSP_SetMotorPWMPulse(s16 l_pulse, s16 r_pulse);
/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#endif /* __MOTOR_H */
