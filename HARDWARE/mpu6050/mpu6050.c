/* Includes ------------------------------------------------------------------*/
#include <math.h>
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
 * @brief Starting sampling rate.
 */
#define DEFAULT_MPU6050_HZ                  (100U)

/**
 * @brief q30 is 2^30 ( = 1073741824)
 */
#define q30                                 (1073741824.0F)

/**
 * @brief The factor of the radian to the angle.
 *        180/PI = 57.295827908
 */
#define RAD2DEG_FACTOR                      (57.3F)

/**
 * @brief device address for MPU6050
 * @note  This constant value must be the same as st.hw->addr in inv_mpu.c
 */
#define MPU6050_DEV_ADDR                    0x68u

/** @defgroup MPU6050_Private_Constants_Register_Address MPU6050 Register Address
  * @brief register address definition for MPU6050
  * @{
 */
#define MPU6050_FIFO_EN_REG                 0x23u
#define MPU6050_INTBP_CFG_REG               0x37u
#define MPU6050_INT_EN_REG                  0x38u
#define MPU6050_USER_CTRL_REG               0x6Au
#define MPU6050_PWR_MGMT1_REG               0x6Bu
#define MPU6050_PWR_MGMT2_REG               0x6Cu
#define MPU6050_WHOAMI_REG                  0x75u
/**
  * @}
  */

/** @defgroup MPU6050_Private_Constants_Return_Value MPU6050 Return Value
  * @brief define the return value used in the MPU6050 function
  * @{
  */

#define MPU6050_OK                          0U
#define MPU6050_NACK                        1U
#define MPU6050_ID_ERROR                    2U
#define MPU6050_FIFO_READ_ERROR             3U
#define MPU6050_FIFO_NO_QUAT                4U
/**
  * @}
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/**
 * @defgroup MPU6050_Private_Macro MPU6050 Private Macros
 * @{
 */
/**
 * @brief Radian to angle
 * @param rad The radian to convert
 */
#define RAD2DEG(rad)                        ((rad) * RAD2DEG_FACTOR)
/**
  * @}
  */
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
static inline u16 inv_row_2_scale(const signed char *row)
{
    u16 b;

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

static inline u16 inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    u16 scalar;

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
        f32 sens;
        u16 accel_sens;
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
 * @brief  Write one byte to the register of MPU6050
 * 
 * @param  reg  The register to be writen
 * @param  data The data to be writen
 * @return MPU6050 status
 */
static u8 BSP_MPU6050_WriteByte(u8 reg, u8 data)
{
    return BSP_MPU6050_Write(reg, 1, &data);
}

/**
 * @brief  Get the Device ID
 * @return MPU6050's ID
 */
static u8 BSP_MPU6050_Get_DeviceID()
{
    u8 id;

    BSP_MPU6050_Read(MPU6050_WHOAMI_REG, 1, &id);

    return id;
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
 * @brief Software resets the MPU6050
 */
void BSP_MPU6050_SW_Reset()
{
    BSP_MPU6050_WriteByte(MPU6050_PWR_MGMT1_REG, 0x80);
    HAL_Delay(100);
    BSP_MPU6050_WriteByte(MPU6050_PWR_MGMT1_REG, 0x00);
}

/**
 * @brief  Get pitch, roll, yaw angle from DMP 
 * 
 * @note   The frequency of reading the data in the FIFO must be the same as
 *         defined by DEFAULT_MPU6050_HZ, otherwise the read will fail.
 *           -# Too fast: MPU6050 has not been sampled, no data is available in the FIFO.
 *           -# Too slow: The FIFO overflows. And the FIFO buffer is emptied in dmp_read_fifo();
 * 
 * @note   This function may take 6ms in 72MHz chip.
 * 
 * @param[out] pitch Angle of pitch
 * @param[out] roll  Angle of roll
 * @param[out] yaw   Angle of yaw
 * @retval     The value can be:
 *               -# @ref MPU6050_OK if successful.
 *               -# @ref MPU6050_FIFO_READ_ERROR if read fifo error.
 *               -# @ref MPU6050_FIFO_NO_QUAT if fifo no quaternion.
 */
u8 BSP_MPU6050_DMP_Get_Angle(f32 *pitch, f32 *roll, f32 *yaw)
{
    u8 res = MPU6050_OK;
    f32 q0 = 0.0f;
    f32 q1 = 0.0f;
    f32 q2 = 0.0f;
    f32 q3 = 0.0f;
    s16 gyro[3], accel[3], sensors;
    unsigned long sensor_timestamp;
    u8 more;
    long quat[4];
    
    if (0 == dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
    {
        if (sensors & INV_WXYZ_QUAT)
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;

            *roll  = RAD2DEG(asin(-2 * q1 * q3 + 2 * q0 * q2));
            *pitch = RAD2DEG(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1));
            *yaw   = RAD2DEG(atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
        }
        else
        {
            /*
             * If we reach this point, the data read from the FIFO
             * most likely not have quaternion.
             */
            res = MPU6050_FIFO_NO_QUAT;
        }
    }
    else
    {
        /*
         * If we reach this point, it indicates that we are 
         * reading the FIFO is too fast or too slowly.
         */
        res = MPU6050_FIFO_READ_ERROR;
    }
    return res;
}

/**
 * @brief  Initialize the DMP of MPU6050 and which IIC bus is it attached to.
 *
 * @retval The value can be:
 *           -# @ref MPU6050_OK if successufl.
 *           -# @ref MPU6050_ID_ERROR if the device id is incorrect.
 */
u8 BSP_MPU6050_Init()
{
    u8 res = MPU6050_OK;

    BSP_MPU6050_HW_Init();
    BSP_MPU6050_SW_Reset();

    /* check device if connected */
    if (MPU6050_DEV_ADDR == BSP_MPU6050_Get_DeviceID())
    {
        /* device is connected */
        mpu_init(NULL);

        /* Get/set hardware configuration. Start gyro. */
        /* Wake up all sensors. */
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        /* Push both gyro and accel data into the FIFO. */
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu_set_sample_rate(DEFAULT_MPU6050_HZ);

        dmp_load_motion_driver_firmware();
        dmp_set_orientation(
            inv_orientation_matrix_to_scalar(gyro_orientation));
        dmp_enable_feature( DMP_FEATURE_6X_LP_QUAT
                          | DMP_FEATURE_TAP
                          | DMP_FEATURE_ANDROID_ORIENT
                          | DMP_FEATURE_SEND_RAW_ACCEL
                          | DMP_FEATURE_SEND_CAL_GYRO
                          | DMP_FEATURE_GYRO_CAL);
        dmp_set_fifo_rate(DEFAULT_MPU6050_HZ);
        mpu_set_dmp_state(1);

        run_self_test();
    }
    else
    {
        /* If we reach this point, we most likely encountered an I2C error. */
        res = MPU6050_ID_ERROR;
    }

    return res;
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