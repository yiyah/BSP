/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct filter
{
    u8  u8Len;
    s16 *ps16data;
    s32  s32Sum;
} FILTER_TypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
s16 UTIL_s16MoveAverageFilter(FILTER_TypeDef *filter, s16 value, u8 filterSize);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#endif
