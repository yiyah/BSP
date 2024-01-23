/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "types.h"
#include "util.h"
#include "iic.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup IIC IIC
  * @brief IIC BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup IIC_Private_typedef IIC Private typedef
  * @{
  */

/**
 * @brief IIC Acknowledge
 */
typedef enum
{
    IIC_ACK     = 0,
    IIC_NACK    = 1
} IIC_ACK_TypeDef;

/**
 * @brief IIC Command
 */
typedef enum
{
    IIC_WRITE   = 0,
    IIC_READ    = 1
} IIC_CMD_TypeDef;

/**
 * @brief IIC Status
 */
typedef enum
{
    IIC_OK      = 0,
    IIC_ERR     = 1
} IIC_STATUS_TypeDef;

/**
  * @} end of IIC_Private_typedef
  */
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/**
 * @defgroup IIC_Private_macro IIC Private macro
 */
/**
 * @brief  Set the IIC SCL pin
 * 
 * @note   This macro must be used after initializing the pin to OD mode.
 * 
 * @param  port: GPIOx
 * @param  pin: GPIO_PIN_x
 * 
 * @retval None
*/
#define IIC_SCL_Set(port, pin)      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

/**
 * @brief  Reset the IIC SCL pin
 * 
 * @note   This macro must be used after initializing the pin to OD mode.
 * 
 * @param  port: GPIOx
 * @param  pin: GPIO_PIN_x
 * 
 * @retval None
*/
#define IIC_SCL_Reset(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

/**
 * @brief  Set the IIC SDA pin
 * 
 * @note   This macro must be used after initializing the pin to OD mode.
 * 
 * @param  port: GPIOx
 * @param  pin: GPIO_PIN_x
 * 
 * @retval None
*/
#define IIC_SDA_Set(port, pin)      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

/**
 * @brief  Reset the IIC SDA pin
 * 
 * @note   This macro must be used after initializing the pin to OD mode.
 * 
 * @param  port: GPIOx
 * @param  pin: GPIO_PIN_x
 * 
 * @retval None
*/
#define IIC_SDA_Reset(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

/**
 * @brief  Read the IIC SDA pin
 * 
 * @param  port: GPIOx
 * @param  pin: GPIO_PIN_x
 * 
 * @retval The value of the IIC SDA pin.
*/
#define IIC_SDA_ReadPin(port, pin)  HAL_GPIO_ReadPin(port, pin)

/**
 * @brief   This macro provides a delay of approximately 5us
 * 
 * @note    This delay will only work properly at a frequency of 72MHz.
 *
 * @details The working principle of this macro is to use NOP() to occupy
 *          one clock cycle of the CPU. Therefore, at a frequency of 72MHz,
 *          the time to execute one instruction is 1/72M.
 *          So, performing __NOP() 72 times takes 1 microsecond.
 *          At the same time, using a for() loop will use jump instructions,
 *          which will add extra time consumption. In order to get closer to
 *          1 microsecond, the number of for() loop iterations should be reduced.
 *          The corresponding NOP() should be added/removed according to the CPU frequency.
 * 
 * @retval  None
*/
#define IIC_Delay()                 do { \
                                        u8 _i; \
                                        for(_i = 0;_i < 5; _i++){ \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
                                            __NOP();__NOP(); \
                                        } \
                                    } while(0)

/**
 * @brief  Check IIC acknowledge if is ack
 * 
 * @note   Be careful not to include this macro in if(). At the same time,
 *         return causes the function that calls the macro to end prematurely.
 * 
 * @retval None
 */
#define CHECK_IIC_ACK(iic_bus)      do { \
                                        if(IIC_NACK == BSP_IIC_readAck(iic_bus)) \
                                        { \
                                            BSP_IIC_Stop(iic_bus); \
                                            return IIC_ERR; \
                                        } \
                                    } while(0)
/**
  * @} end of IIC_Private_macro
  */
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
IIC_BUS_t iic_swBus;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void BSP_IIC_sendAck(IIC_BUS_t *iic)
{
    IIC_SDA_Reset(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
    IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
    IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);      /*!< release SDA */
    IIC_Delay();
}

static void BSP_IIC_sendNAck(IIC_BUS_t *iic)
{
    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
    IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
    IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
}

static void BSP_IIC_Start(IIC_BUS_t *iic)
{
    IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
    IIC_SDA_Reset(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();

    IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
}

static void BSP_IIC_Stop(IIC_BUS_t *iic)
{
    IIC_SDA_Reset(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
    IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
}

static void BSP_IIC_writeByte(IIC_BUS_t *iic, u8 data)
{
    u8 i;

    for(i = 0;i < 8;i++)
    {
        if(0x80 & data)
        {
            IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);
        }
        else
        {
            IIC_SDA_Reset(iic->port_of_data, iic->pin_of_data);
        }
        IIC_Delay();
        IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
        IIC_Delay();
        IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
        data <<= 1;
    }
    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);      /*!< release SDA */
}

static u8 BSP_IIC_readByte(IIC_BUS_t *iic, const IIC_ACK_TypeDef ack)
{
    u8 i;
    u8 data = 0;

    for(i = 0;i < 8;i++)
    {
        data <<= 1;
        IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
        IIC_Delay();

        if(IIC_SDA_ReadPin(iic->port_of_data, iic->pin_of_data))
        {
            data++;
        }
        IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
        IIC_Delay();
    }

    if(IIC_ACK == ack)
    {
        BSP_IIC_sendAck(iic);
    }
    else
    {
        BSP_IIC_sendNAck(iic);
    }
    return data;
}

static IIC_ACK_TypeDef BSP_IIC_readAck(IIC_BUS_t *iic)
{
    IIC_ACK_TypeDef ack = IIC_NACK;

    IIC_SDA_Set(iic->port_of_data, iic->pin_of_data);
    IIC_Delay();
    IIC_SCL_Set(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();
    if(!IIC_SDA_ReadPin(iic->port_of_data, iic->pin_of_data))
    {
        ack = IIC_ACK;
    }
    IIC_SCL_Reset(iic->port_of_clk, iic->pin_of_clk);
    IIC_Delay();

    return ack;
}

static IIC_STATUS_TypeDef BSP_IIC_Write(u8 *devAddr, u8 reg, u8 len, u8 *data)
{
    u8 i;
    IIC_device_t *iic_dev = container_of(devAddr, IIC_device_t, dev_addr);

    BSP_IIC_Start(iic_dev->iic_bus);
    BSP_IIC_writeByte(iic_dev->iic_bus, (*devAddr << 1) | IIC_WRITE);
    CHECK_IIC_ACK(iic_dev->iic_bus);

    BSP_IIC_writeByte(iic_dev->iic_bus, reg);
    CHECK_IIC_ACK(iic_dev->iic_bus);

    for(i = 0;i < len;i++, data++)
    {
        BSP_IIC_writeByte(iic_dev->iic_bus, *data);
        CHECK_IIC_ACK(iic_dev->iic_bus);
    }
    BSP_IIC_Stop(iic_dev->iic_bus);
    return IIC_OK;
}

static IIC_STATUS_TypeDef BSP_IIC_Read(u8 *devAddr, u8 reg, u8 len, u8 *data)
{
    IIC_device_t *iic_dev = container_of(devAddr, IIC_device_t, dev_addr);

    BSP_IIC_Start(iic_dev->iic_bus);
    BSP_IIC_writeByte(iic_dev->iic_bus, (*devAddr << 1) | IIC_WRITE);
    CHECK_IIC_ACK(iic_dev->iic_bus);
    BSP_IIC_writeByte(iic_dev->iic_bus, reg);
    CHECK_IIC_ACK(iic_dev->iic_bus);

    BSP_IIC_Start(iic_dev->iic_bus);
    BSP_IIC_writeByte(iic_dev->iic_bus, (*devAddr << 1) | IIC_READ);
    CHECK_IIC_ACK(iic_dev->iic_bus);
    while (len)
    {
        *data = BSP_IIC_readByte(iic_dev->iic_bus, (len > 1) ? IIC_ACK : IIC_NACK);
        data++;
        len--;
    }
    BSP_IIC_Stop(iic_dev->iic_bus);
    return IIC_OK;
}

/* Exported functions --------------------------------------------------------*/

void BSP_IIC_Init()
{
    iic_swBus.port_of_clk   = GPIOB;
    iic_swBus.pin_of_clk    = GPIO_PIN_8;
    iic_swBus.port_of_data  = GPIOB;
    iic_swBus.pin_of_data   = GPIO_PIN_9;

    iic_swBus.write = BSP_IIC_Write;
    iic_swBus.read  = BSP_IIC_Read;
}

/**
  * @}
  */

/**
  * @}
  */
