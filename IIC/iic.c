#include "iic.h"
#include "util.h"

typedef enum
{
    IIC_ACK     = 0,
    IIC_NACK    = 1
} IIC_ACK_TypeDef;

typedef enum
{
    IIC_WRITE   = 0,
    IIC_READ    = 1
} IIC_CMD_TypeDef;

typedef enum
{
    IIC_OK      = 0,
    IIC_ERR     = 1
} IIC_STATUS_TypeDef;

/** SCL, SDA need to configured as OD mode */
#define IIC_SCL_Set(port, pin)      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define IIC_SCL_Reset(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define IIC_SDA_Set(port, pin)      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define IIC_SDA_Reset(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define IIC_SDA_ReadPin(port, pin)  HAL_GPIO_ReadPin(port, pin)
#define IIC_Delay()                 do{ \
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
                                    }while(0)

#define CHECK_IIC_ACK(iic_bus)      do{ \
                                        if(IIC_NACK == BSP_IIC_readAck(iic_bus)) \
                                        { \
                                            BSP_IIC_Stop(iic_bus); \
                                            return IIC_ERR; \
                                        } \
                                    }while(0)

IIC_BUS_t iic_swBus;

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

void BSP_IIC_Init()
{
    iic_swBus.port_of_clk   = GPIOB;
    iic_swBus.pin_of_clk    = GPIO_PIN_8;
    iic_swBus.port_of_data  = GPIOB;
    iic_swBus.pin_of_data   = GPIO_PIN_9;

    iic_swBus.write = BSP_IIC_Write;
    iic_swBus.read  = BSP_IIC_Read;
}
