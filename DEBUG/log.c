/**
 ******************************************************************************
 * @file    log.c
 * @author  yiyah (yiyah@github.com)
 * @brief 
 * @version 0.1
 * @date    2024-01-31
 * 
 * @copyright Copyright (c) 2024
 *
 ******************************************************************************
 @verbatim
 ==============================================================================
                        ##### DEBGU features #####
 ==============================================================================
 [..] The debug moudle features are:
      (+) support timestamp
      (+) support log level
 
                        ##### How to use this driver #####
 ==============================================================================
 [..]
   (#) step1: Adaptation interfaces.
       (++) Initialize the UART and customize the output function in
            BSP_log_printf();
       
       (++) Define the macro get_tick()

   (#) step2: include log.h and then use the macro log_2(), log_i() ...

   (#) step3: You can see these output in console:
       [603] E/xxxxxxxxxxxxxxx 

 @endverbatim
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdio.h>
#include "types.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "log.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup DEBUG DEBUG
  * @brief DEBUG BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @defgroup DEBUG_Private_Constants DEBUG Private Constants
 * @{
 */

/* Definitions for the size of buffer */
#define SPRINTF_BUF_SIZE            256U   /*!< 256 is the size of u8 */

/**
 * @defgroup DEBUG_Private_Constants_Group1 CSI Constants
 * @brief    Control Sequence Introducer sign
 * @details  More information on https://en.wikipedia.org/wiki/ANSI_escape_code.\n
 *           This part refers to Eassylogger.
 * @{
 */
#ifdef BSP_LOG_COLOR_ENABLE
#define CSI_START                   "\033["
#define CSI_END                     "\033[0m"

/**
 * @defgroup DEBUG_Private_Constants_Group11 Show Style
 * @brief    Output log fonts style
 * @{
 */
#define S_BOLD                      "1m"
#define S_UNDERLINE                 "4m"
#define S_BLINK                     "5m"
#define S_NORMAL                    "22m"
/**
 * @}
*/

/**
 * @defgroup DEBUG_Private_Constants_Group12 Fonts Color
 * @brief    Output log fonts color
 * @{
 */
#define F_BLACK                     "30;"
#define F_RED                       "31;"
#define F_GREEN                     "32;"
#define F_YELLOW                    "33;"
#define F_BLUE                      "34;"
#define F_MAGENTA                   "35;"
#define F_CYAN                      "36;"
#define F_WHITE                     "37;"
/**
 * @}
*/

/**
 * @defgroup DEBUG_Private_Constants_Group13 Background Color
 * @brief    Output log background color
 * @{
 */
#define B_NULL
#define B_BLACK                     "40;"
#define B_RED                       "41;"
#define B_GREEN                     "42;"
#define B_YELLOW                    "43;"
#define B_BLUE                      "44;"
#define B_MAGENTA                   "45;"
#define B_CYAN                      "46;"
#define B_WHITE                     "47;"
/**
 * @}
*/
/**
 * @}
*/

/**
 * @defgroup DEBUG_Private_Constants_Group2 Log Color
 * @brief    Control Sequence Introducer sign
 * @details  Output log default color definition:
 *           [front color] + [background color] + [show style]
 *           @arg front color: @ref DEBUG_Private_Constants_Group12
 *           @arg background color: @ref DEBUG_Private_Constants_Group13
 *           @arg show style: @ref DEBUG_Private_Constants_Group11
 * @{
 */
#define LOG_COLOR_ASSERT            (F_MAGENTA  B_NULL S_NORMAL)
#define LOG_COLOR_ERROR             (F_RED      B_NULL S_BOLD)
#define LOG_COLOR_WARN              (F_YELLOW   B_NULL S_NORMAL)
#define LOG_COLOR_INFO              (F_CYAN     B_NULL S_NORMAL)
#define LOG_COLOR_DEBUG             (F_GREEN    B_NULL S_NORMAL)
#define LOG_COLOR_VERBOSE           (F_BLUE     B_NULL S_NORMAL)
#endif  /* BSP_LOG_COLOR_ENABLE */

/**
 * @}
*/
/**
 * @}
*/
/* Private macro -------------------------------------------------------------*/
/**
 * @defgroup DEBUG_Private_Macros DEBUG Private Macros
 * @{
 */

/**
 * @brief This macro is used to get the tick count of the CPU.
 * @details  Also can use HAL_GetTick() in STM32 HAL driver.\n
 *           Like #define get_tick()       HAL_GetTick()
 */
#define get_tick()          uwTick

/**
 * @}
 */
/* Private variables ---------------------------------------------------------*/
/**
 * @defgroup DEBUG_Private_Variables DEBUG Private Variables
 * @{
 */

/**
 * @brief The buffer to output.
 */
static char sprint_buf[SPRINTF_BUF_SIZE];

/**
 * @brief Level output info
 */
static const char *level_output_info[] = {
        [LOG_LVL_ERROR]     = "E/",
        [LOG_LVL_WARN]      = "W/",
        [LOG_LVL_INFO]      = "I/",
        [LOG_LVL_DEBUG]     = "D/",
        [LOG_LVL_VERBOSE]   = "V/",
};

/**
 * @brief Color output info.
 */
#ifdef BSP_LOG_COLOR_ENABLE
static const char *color_output_info[] = {
        [LOG_LVL_ERROR]     = LOG_COLOR_ERROR,
        [LOG_LVL_WARN]      = LOG_COLOR_WARN,
        [LOG_LVL_INFO]      = LOG_COLOR_INFO,
        [LOG_LVL_DEBUG]     = LOG_COLOR_DEBUG,
        [LOG_LVL_VERBOSE]   = LOG_COLOR_VERBOSE,
};
#endif  /* BSP_LOG_COLOR_ENABLE */

/**
 * @}
 */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @defgroup DEBUG_Private_Functions DEBUG Private Functions
 * @{
 */

/**
 * @brief  Copies the string pointed by the src in to the array pointed by dest,
 *         including the terminating null character.
 * 
 * @param  dest Pointer to the destination array where the content is to be copied.
 * @param  src  Pointer to the string to be copied.
 * @return The length of the copied string.
 */
static u8 log_strcpy(char *dest, const char *src)
{
    const char *src_old = src;

    while (0 != (*dest++ = *src++));
    /*
    while (0 != *src)
    {
        *dest++ = *src++;
        // note that return is (src - src_old) in this way.
    }
    */
    return src - src_old - 1;
}
/**
 * @}
 */
/* Exported functions --------------------------------------------------------*/
/**
 * @defgroup DEBUG_Exported_Functions DEBUG Exported Functions
 * @{
 */

/**
 * @brief   Output the log by UART
 * @details This buffer contain: [color info] + [output info] + [log string].
 * 
 * @param   level The level of log.
 *          This parameter can be one of @arg @ref DEBUG_Exported_Constants_Group1
 * @param   fmt The format string inputed by other module
 * @param   ... args
 */
void BSP_log_printf(u8 level, char *fmt, ...)
{
    va_list args;
    u8 len = 0;
    u32 timestamp = get_tick();

    va_start(args, fmt);

#ifdef BSP_LOG_COLOR_ENABLE
    len += log_strcpy(sprint_buf+len, CSI_START);
    len += log_strcpy(sprint_buf+len, color_output_info[level]);
#endif

    len += sprintf(sprint_buf+len, "[%d] ", timestamp);
    len += log_strcpy(sprint_buf+len, level_output_info[level]);
    len += vsnprintf(sprint_buf+len, SPRINTF_BUF_SIZE, fmt, args);

#ifdef BSP_LOG_COLOR_ENABLE
    len += log_strcpy(sprint_buf+len, CSI_END);
#endif

    va_end(args);

    HAL_UART_Transmit(&huart1, (u8 *)sprint_buf, len, 0xFFFF);
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
