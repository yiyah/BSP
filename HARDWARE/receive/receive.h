/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECEIVE_H
#define __RECEIVE_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define RECEIVE_BUFFER_SIZE         16u
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern u8 RECEIVE_u8Buffer[RECEIVE_BUFFER_SIZE];

/* Exported functions --------------------------------------------------------*/

u8 BSP_RECEIVE_u8GetBuffer(void);
u8 BSP_RECEIVE_u8Parse_Protocol(u8 byte);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#endif
