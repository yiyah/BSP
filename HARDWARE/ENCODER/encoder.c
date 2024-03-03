/**
 ******************************************************************************
 * @file    motor.c
 * @author  yiyah (yiyah@github.com)
 * @brief   ENCODER BSP module driver.
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
   (#) step1: Define a encoder variable with ENCODER_t, and initialize it.
 
   (#) step2: Use BSP_InitEncoder() to start TIMER.

                        ##### About TIMER #####
 ==============================================================================
 [..]
   (#) Please note that need to init TIMER as encoder mode.

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

/** @defgroup ENCODER ENCODER
  * @brief ENCODER BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup ENCODER_Private_typedef ENCODER Private typedef
  * @{
  */

/**
 * @brief Encoder configuration structure definition
 */
struct encoder_s
{
    TIM_HandleTypeDef   *htim;
    u32                 AChannel;
    u32                 BChannel;
};
typedef struct encoder_s ENCODER_t;
/**
  * @}
  */
/* Private define ------------------------------------------------------------*/
/** @defgroup ENCODER_Private_define ENCODER Private define
  * @{
  */
#define LEFT                0U
#define RIGHT               1U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup ENCODER_Private_variables ENCODER Private variables
  * @{
  */
/**
 * @brief The array of encoder parameters
*/
static ENCODER_t encoder[2] = {
    [LEFT] = {&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2},
    [RIGHT] = {&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2}
};
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup ENCODER_Exported_functions ENCODER Exported functions
  * @{
  */

/**
 * @brief Starts the TIMER to enable encoder
 */
void BSP_InitEncoder()
{
    HAL_TIM_Encoder_Start(encoder[LEFT].htim, encoder[LEFT].AChannel);
    HAL_TIM_Encoder_Start(encoder[LEFT].htim, encoder[LEFT].BChannel);
    HAL_TIM_Encoder_Start(encoder[RIGHT].htim, encoder[RIGHT].AChannel);
    HAL_TIM_Encoder_Start(encoder[RIGHT].htim, encoder[RIGHT].BChannel);
}

/**
 * @brief Get the left and right TIMER counter.
 * @note  The parameters have negative, representing the direction.
 * @param[out] l_cnt left counter
 * @param[out] r_cnt right couter
 */
void BSP_Get_Encoder_Count(s16 *l_cnt, s16 *r_cnt)
{
    *l_cnt = __HAL_TIM_GET_COUNTER(encoder[LEFT].htim);
    *r_cnt = __HAL_TIM_GET_COUNTER(encoder[RIGHT].htim);

    if(TRUE == __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder[LEFT].htim))
    {
        *l_cnt = -*l_cnt;
    }
    if(TRUE == __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder[RIGHT].htim))
    {
        *r_cnt = -*r_cnt;
    }
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
