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
