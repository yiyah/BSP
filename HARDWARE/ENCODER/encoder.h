/**
 ******************************************************************************
 * @file    motor.h
 * @author  yiyah (yiyah@github.com)
 * @brief   Header file of ENCODER BSP module.
 * @version 0.1
 * @date    2024-03-03
 * 
 * @copyright Copyright (c) 2024
 * 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BSP_Get_Encoder_Count(l_cnt, r_cnt)                 BSP_Get_Timer_Count(l_cnt, r_cnt, 0)
#define BSP_Get_Encoder_Count_PerUnitTime(l_cnt, r_cnt)     BSP_Get_Timer_Count(l_cnt, r_cnt, 1)
/* Exported functions --------------------------------------------------------*/
/** @addtogroup ENCODER_Exported_functions
  * @{
  */
void BSP_InitEncoder();
void BSP_Get_Timer_Count(s16 *l_cnt, s16 *r_cnt, u8 mode);
void BSP_Encoder_Clear();
/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#endif /* __ENCODER_H */
