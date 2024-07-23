/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "receive.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup RECEIVE receive
  * @brief Receive BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    u8      state;
    u8      lenOfdata;
} RECEIVE_TypdDef;
/* Private define ------------------------------------------------------------*/

#define RECEIVE_STATE_IDLE          0u
#define RECEIVE_STATE_RECEIVING     1u

#define NUM_OF_FRAME_HEAD       (sizeof(g_arrDATA_FRAME_HEAD)/g_arrDATA_FRAME_HEAD[0])
#define NUM_OF_FRAME_END        (sizeof(g_arrDATA_FRAME_END)/g_arrDATA_FRAME_END[0])
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static RECEIVE_TypdDef g_rec;

u8 RECEIVE_u8Buffer[RECEIVE_BUFFER_SIZE];
static u8 g_arrDATA_FRAME_HEAD[] = "";              /* this can be NULL */
static u8 g_arrDATA_FRAME_END[]  = "\n";            /* this cannot be NULL */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

u8 BSP_RECEIVE_u8GetBuffer(void)
{
    s8 ret = 0;

    if (g_rec.state == RECEIVE_STATE_IDLE)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }
    return ret;
}

u8 BSP_RECEIVE_u8Parse_Protocol(u8 byte)
{
    static u8 *pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
    static u8 *pu8BuffWriteIndex = RECEIVE_u8Buffer;

    switch (g_rec.state)
    {
    case RECEIVE_STATE_IDLE:
        if (NUM_OF_FRAME_HEAD > 1)
        {
            /* means there is frame head to parse */
            if (byte == *pu8FrameHeadAndEnd)
            {
                pu8FrameHeadAndEnd++;
                /* means we are receiving frame head */
                if (*pu8FrameHeadAndEnd == '\0')
                {
                    /* means we have received all frame head */
                    pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
                    g_rec.state = RECEIVE_STATE_RECEIVING;

                    /* pont to frame end to check */
                    pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;
                }
                else
                {
                    /* means we are receiving frame head */
                }
            }
            else
            {
                /**
                 * means we are not receiving frame head,
                 * drop this byte and start receiving frame head again.
                 */
                pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
            }
        }
        else
        {
            /**
             * Donot need to parse frame head,
             * the first byte is data.
             */
            *pu8BuffWriteIndex++ = byte;
            g_rec.state = RECEIVE_STATE_RECEIVING;

            /* pont to frame end to check */
            pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;
        }
        break;
    case RECEIVE_STATE_RECEIVING:
        g_rec.lenOfdata = pu8BuffWriteIndex - RECEIVE_u8Buffer;
        if (*pu8FrameHeadAndEnd == byte)
        {
            /* means we are receiving frame end */
            pu8FrameHeadAndEnd++;
            if (*pu8FrameHeadAndEnd == '\0')
            {
                /* means we have received all frame end */
                /* point to frame head for the next receive */
                pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
                pu8BuffWriteIndex = RECEIVE_u8Buffer;
                g_rec.state = RECEIVE_STATE_IDLE;
            }
            else
            {
                /* means we are receiving frame end */
            }
        }
        else if (pu8FrameHeadAndEnd != g_arrDATA_FRAME_END)
        {
            /**
             * means we are recving an error frame end,
             * we need to drop it and receive a new frame.
             */
            pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
            pu8BuffWriteIndex = RECEIVE_u8Buffer;
            g_rec.lenOfdata = 0;
            g_rec.state = RECEIVE_STATE_IDLE;
        }
        else
        {
            if (g_rec.lenOfdata < RECEIVE_BUFFER_SIZE)
            {
                *pu8BuffWriteIndex++ = byte;
            }
            else
            {
                /* buffer is full, drop this byte */
                /* until receive frame end */
                /* keep the last byte to the end */
                *(pu8BuffWriteIndex - 1) = byte;
            }
        }
        break;
    default:
        /* should never run here */
        g_rec.state = RECEIVE_STATE_IDLE;
        break;
    }
    return 0;
}
/**
  * @}
  */

/**
  * @}
  */