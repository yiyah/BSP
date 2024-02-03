/**
 ******************************************************************************
 * @file    log.h
 * @author  yiyah (yiyah@github.com)
 * @brief 
 * @version 0.1
 * @date    2024-02-03
 * 
 * @copyright Copyright (c) 2024
 * 
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOG_H
#define __LOG_H
/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */

/** @addtogroup DEBUG
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Enable the output with color */
#define BSP_LOG_COLOR_ENABLE

/**
 * @defgroup DEBUG_Exported_Constants DEBUG Exported Constants
 * @{
 */

/**
 * @defgroup DEBUG_Exported_Constants_Group1 DEBUG Log Level
 * @brief    Define the log level constants
 * @{
 */
#define LOG_LVL_ERROR           0U
#define LOG_LVL_WARN            1U
#define LOG_LVL_INFO            2U
#define LOG_LVL_DEBUG           3U
#define LOG_LVL_VERBOSE         4U
/**
  * @}
  */
/**
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/**
 * @defgroup DEBUG_Exported_Macros DEBUG Exported Macro
 * @{
 */
#define BSP_log_e(fmt, ...)     BSP_log_printf(LOG_LVL_ERROR, fmt, ##__VA_ARGS__)
#define BSP_log_w(fmt, ...)     BSP_log_printf(LOG_LVL_WARN, fmt, ##__VA_ARGS__)
#define BSP_log_i(fmt, ...)     BSP_log_printf(LOG_LVL_INFO, fmt, ##__VA_ARGS__)
#define BSP_log_d(fmt, ...)     BSP_log_printf(LOG_LVL_DEBUG, fmt, ##__VA_ARGS__)
#define BSP_log_v(fmt, ...)     BSP_log_printf(LOG_LVL_VERBOSE, fmt, ##__VA_ARGS__)

#define log_e(fmt, ...)         BSP_log_e(LOG_TAG fmt, ##__VA_ARGS__)
#define log_w(fmt, ...)         BSP_log_w(LOG_TAG fmt, ##__VA_ARGS__)
#define log_i(fmt, ...)         BSP_log_i(LOG_TAG fmt, ##__VA_ARGS__)
#define log_d(fmt, ...)         BSP_log_d(LOG_TAG fmt, ##__VA_ARGS__)
#define log_v(fmt, ...)         BSP_log_v(LOG_TAG fmt, ##__VA_ARGS__)
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/**
 * @addtogroup DEBUG_Exported_Functions
 * @{
 */
void BSP_log_printf(u8 level, char *fmt, ...);
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
