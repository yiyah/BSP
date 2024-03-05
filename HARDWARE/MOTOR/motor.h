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
/** @defgroup MOTOR_Exported_Constants MOTOR Exported Constants
  * @{
  */
#define MOTOR_PWM_ARR       (7200U - 1U)        /*!< need to sync with timer's period */
/**
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/** @defgroup MOTOR_Exported_Macros MOTOR Exported Macros
  * @{
  */

/**
 * @brief Convert percentages into float
 * @param[in] percentage the value can be [0, 100]
 */
#define FROM_PERCENTAGE(percentage)     ((percentage) * 0.01)

/**
 * @brief Set the PWM of the motor
 * @param[in] l the PWM of left motor
 * @param[in] r the PWM of right motor
 */
#define BSP_SetMotorPWM(l, r) BSP_SetMotorPWMPulse((s16)(FROM_PERCENTAGE(l) * MOTOR_PWM_ARR), \
                                                   (s16)(FROM_PERCENTAGE(r) * MOTOR_PWM_ARR))
/**
  * @}
  */
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
