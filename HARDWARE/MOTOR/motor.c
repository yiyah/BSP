/**
 ******************************************************************************
 * @file    motor.c
 * @author  yiyah (yiyah@github.com)
 * @brief   MOTOR BSP module driver.
 * @version 0.1
 * @date    2024-03-03
 * 
 * @copyright Copyright (c) 2024
 * 
 ******************************************************************************
 @verbatim
 ==============================================================================
                        ##### How to use this driver #####
 ==============================================================================
 [..]
   (#) step1: Define a motor variable with MOTOR_t, and initialize it.
 
   (#) step2: Use BSP_SetMotorPWMPulse() to control motor.

                        ##### About Pin #####
 ==============================================================================
 [..]
   (#) Please note that need to init GPIO(IN1/IN2) and TIMER(PWM)
       before use this driver !!!

 @endverbatim
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "types.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup MOTOR MOTOR
  * @brief MOTOR BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup MOTOR_Private_typedef MOTOR Private typedef
  * @{
  */
 /**
 * @brief TB6612 input definition
 */
typedef struct tb6612_input_s
{
    GPIO_TypeDef    *GPIOx;
    u16             GPIO_Pin;
} TB6612_INPUT_T ;

/**
 * @brief Motor paramters definition
 */
typedef struct motor_s
{
    TIM_HandleTypeDef   *htim;
    u32                 channel;
    TB6612_INPUT_T      IN1;
    TB6612_INPUT_T      IN2;
    u8                  tire_Radius_cm;
} MOTOR_t;
/**
  * @}
  */
/* Private define ------------------------------------------------------------*/
/** @defgroup MOTOR_Private_Constants MOTOR Private Constants
  * @{
  */
#define LEFT                0U
#define RIGHT               1U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup MOTOR_Private_variables MOTOR Private variables
  * @{
  */
/**
 * @brief The array of motor parameters
*/
static MOTOR_t motor[2] = {
    [LEFT]  = {&htim1, TIM_CHANNEL_4,                /*!< PA11 */
                {GPIOB, GPIO_PIN_13}, {GPIOB, GPIO_PIN_12},
               8},
    [RIGHT] = {&htim1, TIM_CHANNEL_1,                /*!< PA8 */
                {GPIOB, GPIO_PIN_14}, {GPIOB, GPIO_PIN_15},
               8}
};
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup MOTOR_Exported_functions MOTOR Exported functions
  * @{
  */
/**
 * @brief Starts the left & right motor PWM signal generation
 */
void BSP_InitMotor()
{
    HAL_TIM_PWM_Start(motor[LEFT].htim, motor[LEFT].channel);
    HAL_TIM_PWM_Start(motor[RIGHT].htim, motor[RIGHT].channel);
}

/**
 * @brief Stops the left & right motor PWM signal generation.
 */
void BSP_StopMotor()
{
    HAL_GPIO_WritePin(motor[LEFT].IN1.GPIOx, motor[LEFT].IN1.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor[LEFT].IN2.GPIOx, motor[LEFT].IN2.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor[RIGHT].IN1.GPIOx, motor[RIGHT].IN1.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor[RIGHT].IN2.GPIOx, motor[RIGHT].IN2.GPIO_Pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Stop(motor[LEFT].htim, motor[LEFT].channel);
    HAL_TIM_PWM_Stop(motor[RIGHT].htim, motor[RIGHT].channel);
}

/**
 * @brief Set the PWM pulse value of left and right motor.
 * 
 * @param l_pulse specifies the left motor PWM pulse value.
 *        @arg The range of values can be: [0, @ref MOTOR_PWM_ARR].
 * @param r_pulse specifies the right motor PWM pulse value.
 *        @arg The range of values can be: [0, @ref MOTOR_PWM_ARR].
 */
void BSP_SetMotorPWMPulse(s16 l_pulse, s16 r_pulse)
{
    /* left motor */
    if(0 <= l_pulse)
    {
        HAL_GPIO_WritePin(motor[LEFT].IN1.GPIOx, motor[LEFT].IN1.GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[LEFT].IN2.GPIOx, motor[LEFT].IN2.GPIO_Pin, GPIO_PIN_RESET);
    }
    else
    {   l_pulse = -l_pulse;
        HAL_GPIO_WritePin(motor[LEFT].IN1.GPIOx, motor[LEFT].IN1.GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[LEFT].IN2.GPIOx, motor[LEFT].IN2.GPIO_Pin, GPIO_PIN_SET);
    }
    /* right motor */
    if(0 <= r_pulse)
    {
        HAL_GPIO_WritePin(motor[RIGHT].IN1.GPIOx, motor[RIGHT].IN1.GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[RIGHT].IN2.GPIOx, motor[RIGHT].IN2.GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        r_pulse = -r_pulse;
        HAL_GPIO_WritePin(motor[RIGHT].IN1.GPIOx, motor[RIGHT].IN1.GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[RIGHT].IN2.GPIOx, motor[RIGHT].IN2.GPIO_Pin, GPIO_PIN_SET);
    }
    /* set pwm pulse */
    __HAL_TIM_SET_COMPARE(motor[LEFT].htim, motor[LEFT].channel, l_pulse);
    __HAL_TIM_SET_COMPARE(motor[RIGHT].htim, motor[RIGHT].channel, r_pulse);
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
