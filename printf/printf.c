/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Retargets the C library printf function to the USART.
  *
  * @param  None
  *
  * @note   This function only use in #include <stdio.h>
  *
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (u8 *)&ch, 1, 0xFFFF);
  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
  return ch;
}

/* Exported functions --------------------------------------------------------*/
// #define USE_BSP_PRINTF
#ifdef USE_BSP_PRINTF
#include <stdarg.h>
#include <stdio.h>          // for vsnprintf()
#define PRINTF_BUFFER_SIZE 128
char _PRINTF_BUFFER[PRINTF_BUFFER_SIZE];
void BSP_printf(const char *fmt, ...)
{
    u8 len = 0;
    va_list ap;

    va_start(ap, fmt);
    // len = vsnprintf(_PRINTF_BUFFER, PRINTF_BUFFER_SIZE, fmt, ap);
    for(u8 i = 0; i < PRINTF_BUFFER_SIZE; i++)
    {
        if('%' == *fmt)
        {

        }
        else
        {
            len++;
            _PRINTF_BUFFER[i] = *fmt;
            if (*fmt++ == 0)
            {
                _PRINTF_BUFFER[i+1] = 0;
                break;
            }
        }
    }

    va_end(ap);

    HAL_UART_Transmit(&huart1, (u8 *)_PRINTF_BUFFER, len, 0xFFFF);
}
#endif
