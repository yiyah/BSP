/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"      /*!< for include in inv_mpu.c */

/** @addtogroup BSP
  * @{
  */

/** @addtogroup MPU6050
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup MPU6050_Exported_Function
  * @{
  */

u8 BSP_MPU6050_Init();
u8 BSP_MPU6050_Write(u8 reg, u8 len, u8 *data);
u8 BSP_MPU6050_Read(u8 reg, u8 len, u8 *data);
u8 BSP_MPU6050_DMP_Get_Angle(float *pitch, float *roll, float *yaw);
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
