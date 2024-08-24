/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct filter
{
    u8  u8Len;
    f32 *pf32data;
    f32  f32Sum;
} FILTER_TypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
f32 UTIL_f32MoveAverageFilter(FILTER_TypeDef *filter, f32 value, u8 filterSize);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#endif
