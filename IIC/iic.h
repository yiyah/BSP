#ifndef __IIC_H
#define __IIC_H

#include "types.h"
#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"


typedef struct IIC_BUS_s
{
    u8 num_of_devices;
    GPIO_TypeDef *port_of_clk;
    u16 pin_of_clk;
    GPIO_TypeDef *port_of_data;
    u16 pin_of_data;

    u8 (*write)(u8 *devAddr, u8 reg, u8 len, u8 *data);
    u8 (*read)(u8 *devAddr, u8 reg, u8 len, u8 *data);
} IIC_BUS_t;

typedef struct IIC_device_s
{
    u8 dev_addr;
    IIC_BUS_t *iic_bus;
} IIC_device_t;


extern IIC_BUS_t iic_swBus;

#endif
