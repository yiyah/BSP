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
    u8      *pu8FrameHeadAndEnd;
    u8      *pu8BuffWriteIndex;
} RECEIVE_TypdDef;
/* Private define ------------------------------------------------------------*/

#define RECEIVE_BUFFER_SIZE         16U

#define REC_STA_REMAIN              0U          /* never use this */
#define REC_STA_INIT                1U
#define REC_STA_IDLE                2U
#define REC_STA_IDLE_NOT_READED     3U
#define REC_STA_RECEIVING_HEAD      4U
#define REC_STA_RECEIVING_DATA      5U
#define REC_STA_RECEIVING_END       6U

#define ENABLE_FRAME_HEAD           FALSE
#define NUM_OF_FRAME_HEAD       (sizeof(g_arrDATA_FRAME_HEAD)/g_arrDATA_FRAME_HEAD[0])
#define NUM_OF_FRAME_END        (sizeof(g_arrDATA_FRAME_END)/g_arrDATA_FRAME_END[0])
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static RECEIVE_TypdDef g_rec;

static u8 g_u8Buffer[RECEIVE_BUFFER_SIZE];

#if (ENABLE_FRAME_HEAD == TRUE)
static u8 g_arrDATA_FRAME_HEAD[] = "$$";              /* this can be NULL */
#endif  /* (ENABLE_FRAME_HEAD == TRUE) */
static u8 g_arrDATA_FRAME_END[]  = "\n";            /* this cannot be NULL */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void onIDLEState(u8 byte)
{
    g_rec.pu8BuffWriteIndex = g_u8Buffer;
#if (ENABLE_FRAME_HEAD == TRUE)
    g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
    g_rec.state = REC_STA_RECEIVING_HEAD;
    *g_rec.pu8BuffWriteIndex = byte;
#else
    g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;
    g_rec.state = REC_STA_RECEIVING_DATA;
    *g_rec.pu8BuffWriteIndex++ = byte;
#endif
}

#if (ENABLE_FRAME_HEAD == TRUE)
void onReceivingFrameHead(u8 byte)
{
    /* there is last byte of frame head in buffer */
    if (*g_rec.pu8BuffWriteIndex == *g_rec.pu8FrameHeadAndEnd)
    {
        /* means the first frame head is received, but we also
           need to continue to check if there are still frame headers.
         */
        g_rec.pu8FrameHeadAndEnd++;
        if (*g_rec.pu8FrameHeadAndEnd == '\0')
        {
            /* means all frame head is received */
            /* this byte is data byte */
            *g_rec.pu8BuffWriteIndex++ = byte;

            /* set the pointer for parse frame end */
            g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;

            g_rec.state = REC_STA_RECEIVING_DATA;
        }
        else
        {
            /* means there are still frame head to receive,
               and save the byte this received for next check.
             */
            *g_rec.pu8BuffWriteIndex = byte;
        }
    }
    else
    {
        /* means receive a error frame head */
        /* set the pointer for next parse frame head */
        g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
    }
}
#endif

void onReceivingDATA(u8 byte)
{
    g_rec.lenOfdata = g_rec.pu8BuffWriteIndex - g_u8Buffer;

    if (*g_rec.pu8FrameHeadAndEnd == byte)
    {
        /* means we are receiving frame end */
        g_rec.pu8FrameHeadAndEnd++;
        if (*g_rec.pu8FrameHeadAndEnd == '\0')
        {
            /**
               means there is one byte frame end,
               and it is correct, than we had finished receiving.
             */
            #if (ENABLE_FRAME_HEAD == TRUE)
            /* point to frame head for the next receive */
            g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
            #else
            /* point to frame end for the next receive */
            g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;
            #endif
            g_rec.pu8BuffWriteIndex = g_u8Buffer;
            g_rec.state = REC_STA_IDLE_NOT_READED;
        }
        else
        {
            /* means there is more than one byte frame end,
               and we need to change state to parse.
             */
            g_rec.state = REC_STA_RECEIVING_END;
        }
    }
    else
    {
        if (g_rec.lenOfdata < RECEIVE_BUFFER_SIZE)
        {
            *g_rec.pu8BuffWriteIndex++ = byte;
        }
        else
        {
            /* buffer is full, drop this byte */
            /* until receive frame end */
            /* keep the last byte to the end */
            *(g_rec.pu8BuffWriteIndex - 1) = byte;
        }
    }
}

void onReceivingFrameEnd(u8 byte)
{
    if (*g_rec.pu8FrameHeadAndEnd == byte)
    {
        /* means we are receiving frame end */
        g_rec.pu8FrameHeadAndEnd++;
        if (*g_rec.pu8FrameHeadAndEnd == '\0')
        {
            /**
               means we had finished receiving.
             */
            g_rec.pu8BuffWriteIndex = g_u8Buffer;
            g_rec.state = REC_STA_IDLE_NOT_READED;
        }
        else
        {
            /* means we should keep receiving frame end */
        }
    }
    else
    {
        /**
         * means we are recving an error frame end,
         * we need to drop it and receive a new frame.
         */
        g_rec.pu8BuffWriteIndex = g_u8Buffer;
        g_rec.lenOfdata = 0;
        g_rec.state = REC_STA_IDLE;
    }
}

/* Exported functions --------------------------------------------------------*/

void BSP_RECEIVE_vInit(void)
{
    g_rec.state = REC_STA_INIT;
    g_rec.lenOfdata = 0;
    g_rec.pu8BuffWriteIndex = g_u8Buffer;
#if (ENABLE_FRAME_HEAD == TRUE)
    g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_HEAD;
#else
    g_rec.pu8FrameHeadAndEnd = g_arrDATA_FRAME_END;
#endif
}

/**
 * @brief Get the receiver buffer
 * 
 * @param[out] pdata the data where to point
 * @return u8 The return value can be below values
 *          @arg -1: means the receive buffer is not ready
 *          @arg ret: the length of data
 */
s8 BSP_RECEIVE_u8GetBuffer(u8 **pdata)
{
    s8 ret = -1;

    if (g_rec.state == REC_STA_IDLE_NOT_READED)
    {
        *pdata = g_u8Buffer;
        ret = g_rec.lenOfdata;
        g_rec.state = REC_STA_IDLE;
    }
    else
    {
        ret = -1;
    }
    return ret;
}

u8 BSP_RECEIVE_u8Parse_Protocol(u8 byte)
{
    switch (g_rec.state)
    {
    case REC_STA_INIT:
    case REC_STA_IDLE:
    case REC_STA_IDLE_NOT_READED:   /* means we would drop the older data */
        onIDLEState(byte);
        break;
    #if (ENABLE_FRAME_HEAD == TRUE)
    case REC_STA_RECEIVING_HEAD:
        onReceivingFrameHead(byte);
        break;
    #endif
    case REC_STA_RECEIVING_DATA:
        onReceivingDATA(byte);
        break;
    case REC_STA_RECEIVING_END:
        onReceivingFrameEnd(byte);
        break;
    default:
        /* should never run here */
        g_rec.state = REC_STA_INIT;
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