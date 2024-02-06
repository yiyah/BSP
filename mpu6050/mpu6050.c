/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"                  /*!< for HAL_Delay() */
#include "types.h"
#include "iic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"

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

/**
 * @brief Starting sampling rate.
 */
#define DEFAULT_MPU_HZ                      (100U)

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
/** @defgroup MPU6050_Private_Variables MPU6050 Private Variables
  * @{
  */
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
/**
  * @}
  */
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
/* Private functions ---------------------------------------------------------*/
/**
 * @defgroup MPU6050_Private_Function MPU6050 Private Function
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


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;              /* error */
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/**
 * @brief run self test
 */
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
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
 * @brief  Initialize the DMP of MPU6050 and which IIC bus is it attached to.
 *
 * @return 0 if successufl.
 */
u8 BSP_MPU6050_Init()
{
    u8 res = 0;

    BSP_MPU6050_HW_Init();

    res += mpu_init(NULL);

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    res += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    res += mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    res += mpu_set_sample_rate(DEFAULT_MPU_HZ);

    res += dmp_load_motion_driver_firmware();
    res += dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
    res += dmp_enable_feature( DMP_FEATURE_6X_LP_QUAT
                      | DMP_FEATURE_TAP
                      | DMP_FEATURE_ANDROID_ORIENT
                      | DMP_FEATURE_SEND_RAW_ACCEL
                      | DMP_FEATURE_SEND_CAL_GYRO
                      | DMP_FEATURE_GYRO_CAL);
    res += dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    res += mpu_set_dmp_state(1);
    res += mpu_set_gyro_fsr(250);

    /* run_self_test(); */

    return ((res == 0) ? 0  : 1);
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