/****************************************************************************
 * @file    : drivers/mpu6050/mpu6500_reg.h
 * @author  : Pierre-noel Bouteville <pnb990@gmail.com>
            : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/06
 ****************************************************************************/

#ifndef _DRIVERS_SENSOR_INV_MPU6050_H_
#define _DRIVERS_SENSOR_INV_MPU6050_H_


#define INV_MPU_SELFTEST_X      0x0D
#define INV_MPU_SELFTEST_Y      0x0E
#define INV_MPU_SELFTEST_Z      0x0F
#define INV_MPU_SELFTEST_A      0x10

#define INV_MPU_WHO_AM_I        0x75
#define INV_MPU_RATE_DIV        0x19
#define INV_MPU_LPF             0x1A
#define INV_MPU_PROD_ID         0x0C
#define INV_MPU_USER_CTRL       0x6A
#define INV_MPU_FIFO_EN         0x23
#define INV_MPU_GYRO_CFG        0x1B
#define INV_MPU_ACCEL_CFG       0x1C
#define INV_MPU_MOTION_THR      0x1F
#define INV_MPU_MOTION_DUR      0x20
#define INV_MPU_FIFO_COUNT_H    0x72
#define INV_MPU_FIFO_R_W        0x74
#define INV_MPU_RAW_GYRO        0x43
#define INV_MPU_RAW_ACCEL       0x3B
#define INV_MPU_TEMP            0x41
#define INV_MPU_INT_ENABLE      0x38
#define INV_MPU_DMP_INT_STATUS  0x39
#define INV_MPU_INT_STATUS      0x3A
#define INV_MPU_PWR_MGMT_1      0x6B
#define INV_MPU_PWR_MGMT_2      0x6C
#define INV_MPU_INT_PIN_CFG     0x37
#define INV_MPU_MEM_R_W         0x6F
#define INV_MPU_ACCEL_OFFS      0x06
#define INV_MPU_I2C_MST         0x24
#define INV_MPU_BANK_SEL        0x6D
#define INV_MPU_MEM_START_ADDR  0x6E
#define INV_MPU_PRGM_START_H    0x70

/* Gyroscope offset */

#define INV_MPU_XG_OFFSET_H     0x13
#define INV_MPU_XG_OFFSET_L     0x14
#define INV_MPU_YG_OFFSET_H     0x15
#define INV_MPU_YG_OFFSET_L     0x16
#define INV_MPU_ZG_OFFSET_H     0x17
#define INV_MPU_ZG_OFFSET_L     0x18

/* Accelerometer offset */

#define INV_MPU_XA_OFFSET_H     0x06
#define INV_MPU_XA_OFFSET_L     0x07
#define INV_MPU_YA_OFFSET_H     0x08
#define INV_MPU_YA_OFFSET_L     0x09
#define INV_MPU_ZA_OFFSET_H     0x0A
#define INV_MPU_ZA_OFFSET_L     0x0B

/* Hardware Value *************************************************************/

#define INV_MPU_MAX_FIFO    1024
#define INV_MPU_NUM_REG     118
#define INV_MPU_TEMP_SENS   340
#define INV_MPU_TEMP_OFFSET -521
#define INV_MPU_BANK_SIZE   256

/* TEST Value *****************************************************************/

#define INV_MPU_GYRO_SENS       (32768/250)
#define INV_MPU_ACCEL_SENS      (32768/16)
#define INV_MPU_REG_RATE_DIV    (0)    /* 1kHz. */
#define INV_MPU_REG_LPF         (1)    /* 188Hz. */
#define INV_MPU_REG_GYRO_FSR    (0)    /* 250dps. */
#define INV_MPU_REG_ACCEL_FSR   (0x18) /* 16g. */
#define INV_MPU_PACKET_THRESH   (5)    /* 5% */
#define INV_MPU_WAIT_MS         (50)

/* Criteria A */
#define INV_MPU_MAX_ACCEL_VAR   (0.14f)
#define INV_MPU_MAX_GYRO_VAR    (0.14f)

/* Criteria B */
#define INV_MPU_MIN_ACCEL_G     (0.3f)
#define INV_MPU_MAX_ACCEL_G     (0.95f)
#define INV_MPU_MAX_GYRO_DPS    (105.f)

/* Criteria C */
#define INV_MPU_MIN_DPS         (10.f)

#define HWST_MAX_PACKET_LENGTH  (512)

#endif  /* _DRIVERS_SENSOR_INV_MPU6050_H_ */

