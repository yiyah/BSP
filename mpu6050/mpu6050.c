/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"                  // for HAL_Delay()
#include "mpu6050.h"
#include "iic.h"
#include "inv_mpu.h"

/** @addtogroup BSP
  * @{
  */
/** @defgroup MPU6050 MPU6050
  * @brief MPU6050 BSP driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup MPU6050_Private_Constants MPU6050 Private Constants
  * @{
  */

/**
 * @brief device address for MPU6050
 * @note  This constant value must be the same as st.hw->addr in inv_mpu.c
 */
#define MPU6050_DEV_ADDR                    0x68

/** @defgroup MPU6050_Private_Constants_Return_Value MPU6050 Return Value
  * @brief define the return value used in the MPU6050 function
  * @{
  */

#define MPU6050_OK                          0U
#define MPU6050_NACK                        1U

/**
  * @}
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/**
 * @defgroup MPU6050_Exported_Variables MPU6050 Exported Variables
 *
@verbatim
 ===============================================================================
                      ##### MPU6050 Exported Variables #####
 ===============================================================================
  [..]
    This variable should be only have one.

@endverbatim
 * @{
 */

/**
 * @brief    define an object to read and write to the MPU6050
 * 
 * @details  usage: mpu6050.write(reg, len, *data);
 *                  mpu6050.read(reg, len, *data);
*/
IIC_device_t mpu6050;

/**
 * @}
 */
/* Private function prototypes -----------------------------------------------*/
/**
 * @defgroup MPU6050_Private_Function MPU6050 Private Function
 * @{
 */

static void BSP_MPU6050_HW_Init();

/**
 * @}
 */
/* Private functions ---------------------------------------------------------*/
/**
 * @addtogroup MPU6050_Private_Function MPU6050 Private Function
 * @{
 */

/**
 * @brief Initialize which IIC bus is the MPU6050 attached to
*/
static void BSP_MPU6050_HW_Init()
{
    mpu6050.dev_addr = MPU6050_DEV_ADDR;
    mpu6050.iic_bus = &iic_swBus;
    mpu6050.iic_bus->num_of_devices++;
}

/**
 * @}
 */
/* Exported functions --------------------------------------------------------*/
/**
 * @defgroup MPU6050_Exported_Function MPU6050 Exported Function
 * @{
 */

/**
 * @brief     Write data to the specified MPU6050 register
 * 
 * @param[in] reg  the register to be written
 * @param[in] len  the length of the data
 * @param[in] data pointer to the data to be written
 * @return    MPU6050 status
 */
u8 BSP_MPU6050_Write(u8 reg, u8 len, u8 *data)
{
    u8 status = MPU6050_OK;
    if(mpu6050.iic_bus->write(&mpu6050.dev_addr, reg, len, data))
    {
        status = MPU6050_NACK;
    }

    return status;
}

/**
 * @brief     Read data from the specified MPU6050 register
 * 
 * @param[in] reg  the register to be reaed
 * @param[in] len  the length of the data
 * @param[in] data pointer to the data to be read
 * @return    MPU6050 status
 */
u8 BSP_MPU6050_Read(u8 reg, u8 len, u8 *data)
{
    u8 status = MPU6050_OK;

    if(mpu6050.iic_bus->read(&mpu6050.dev_addr, reg, len, data))
    {
        status = MPU6050_NACK;
    }
    return status;
}

/**
 * @brief Initialize the register of the MPU6050 and which IIC bus is it attached to
 */
void BSP_MPU6050_Init()
{
    BSP_MPU6050_HW_Init();
    mpu_init(NULL);
}

/**
  * @}
  */
/**
  * @}
  */
/**
  * @}
  */