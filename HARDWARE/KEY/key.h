/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEY_H
#define __KEY_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported class ------------------------------------------------------------*/
class cGPIO
{
private:
    /* data */
public:
    void write(GPIO_PinState state);
};

#ifdef __cplusplus
extern "C" {
#endif

void gpio_write(GPIO_PinState state);

#ifdef __cplusplus
}
#endif


/* Exported functions --------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#endif
