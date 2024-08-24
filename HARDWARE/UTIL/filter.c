/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "filter.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup FILTER FILTER
  * @brief FILTER BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

f32 UTIL_f32MoveAverageFilter(FILTER_TypeDef *filter, f32 value, u8 filterSize)
{
    f32 res = 0.0F;

    /* sum minus the first saved value */
    filter->f32Sum -= filter->pf32data[filterSize-1];

    /* Move all historical values back one bit, bit 0 is the incoming value */
    for (u8 i = filterSize - 1; i >= 1; i--)
    {
        filter->pf32data[i] = filter->pf32data[i-1];
    }
    filter->pf32data[0] = value;

    /* the new value in the first index */
    filter->f32Sum += filter->pf32data[0];
    
    if (filter->u8Len < filterSize)
    {
        filter->u8Len++;
    }
    else
    {
        /* keep the filter->u8Len equal to filterSize */
    }
    res = filter->f32Sum / filter->u8Len;

    /* Rounding */
    if (res > 0.0)
    {
        res += 0.5;
    }
    else
    {
        res -= 0.5;
    }
    return res;
}

/**
  * @}
  */

/**
  * @}
  */
