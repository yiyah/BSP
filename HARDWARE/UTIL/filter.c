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

s16 UTIL_s16MoveAverageFilter(FILTER_TypeDef *filter, s16 value, u8 filterSize)
{
    /* sum minus the first saved value */
    filter->s32Sum -= filter->ps16data[filterSize-1];

    /* Move all historical values back one bit, bit 0 is the incoming value */
    for (u8 i = filterSize - 1; i >= 1; i--)
    {
        filter->ps16data[i] = filter->ps16data[i-1];
    }
    filter->ps16data[0] = value;

    /* the new value in the first index */
    filter->s32Sum += filter->ps16data[0];
    
    if (filter->u8Len < filterSize)
    {
        filter->u8Len++;
    }
    else
    {
        /* keep the filter->u8Len equal to filterSize */
    }

    return (s16)((filter->s32Sum / filter->u8Len)+0.5);
}

/**
  * @}
  */

/**
  * @}
  */
