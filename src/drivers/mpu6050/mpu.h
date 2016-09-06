/****************************************************************************
 * @file    : drivers/mpu6050/mpu.c
 * @author  : Pierre-noel Bouteville <pnb990@gmail.com>
            : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/06
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_INV_MPU_H
#define __INCLUDE_NUTTX_INPUT_INV_MPU_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * Settings that effect the driver: CONFIG_DISABLE_POLL
 *
 * CONFIG_INVENSENSE_SPI
 *   Enables support for the SPI interface (not currenly supported)
 * CONFIG_INVENSENSE_I2C
 *   Enables support for the I2C interface
 */

/* The Invensence interfaces with the target CPU via a I2C or SPI interface. 
 */

#if !defined(CONFIG_INVENSENSE_I2C) && !defined(CONFIG_INVENSENSE_SPI)
#  error "One of CONFIG_INVENSENSE_I2C or CONFIG_INVENSENSE_SPI must be defined"
#endif

#if defined(CONFIG_INVENSENSE_I2C) && defined(CONFIG_INVENSENSE_SPI)
#  error "Only one of CONFIG_INVENSENSE_I2C or CONFIG_INVENSENSE_SPI can be defined"
#endif

/* Register bit definitions */

/* Enable bit fields */

#define MPU_X_GYRO      (0x40)
#define MPU_Y_GYRO      (0x20)
#define MPU_Z_GYRO      (0x10)
#define MPU_XYZ_GYRO    (MPU_X_GYRO | MPU_Y_GYRO | MPU_Z_GYRO)
#define MPU_XYZ_ACCEL   (0x08)
#define MPU_XYZ_COMPASS (0x01)

/* Interrupt bit fields */

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/* IOCTL Commands ***********************************************************/
/* Invensence MPU IOCTL commands. 
 */

/* MPU_ENABLE Enable sensors
 *
 * Argument: enable bit fields.
 */

#define MPU_ENABLE          _SNIOC(0x0001)

/* MPU_FREQUENCY set sensors acquisition frequency.
 *
 * Argument: frequency in Hz.
 */

#define MPU_FREQUENCY       _SNIOC(0x0002)

/* MPU_GET_TEMP read temperature sensors.
 *
 * Argument: pointer to int32_t filled with temperature in degree.
 */

#define MPU_GET_TEMP        _SNIOC(0x0003)


/* MPU_RESET_FIFO reset mpu fifo.
 *
 * Argument: None
 *
 */

#define MPU_RESET_FIFO      _SNIOC(0x0004)


/* MPU_SET_ACCEL_OFF set accelerometer offset.
 *
 * Argument: struct mpu_axes_s
 *
 */

#define MPU_SET_ACCEL_OFF   _SNIOC(0x0005)


/* MPU_SET_GYRO_OFF set gyroscope offset.
 *
 * Argument: struct mpu_axes_s
 *
 */

#define MPU_SET_GYRO_OFF    _SNIOC(0x0006)


/* MPU_GET_ACCEL_OFF set accelerometer offset.
 *
 * Argument: struct mpu_axes_s
 *
 */

#define MPU_GET_ACCEL_OFF   _SNIOC(0x0007)


/* MPU_GET_GYRO_OFF set gyroscope offset.
 *
 * Argument: struct mpu_axes_s
 *
 */

#define MPU_GET_GYRO_OFF    _SNIOC(0x0008)


/* MPU_CALIBRATION do a accelerometer and gyroscope calibrated offset.
 *
 * Argument: struct mpu_axes_s
 *
 */

#define MPU_CALIBRATION     _SNIOC(0x0009)

/* MPU_DUMP_REG dump all register.
 *
 * Argument: None
 *
 */

#define MPU_DUMP_REG        _SNIOC(0x000A)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

enum mpu_axes_e
{
    MPU_AXES_ACC,
    MPU_AXES_GYRO,
    MPU_AXES_COMP,
    MPU_AXES_NBR
};

struct mpu_firmware_s {

    /* address in invensense IC */

    uint32_t start_addr;

    /* sample rate of firmware */

    uint32_t sample_rate;

    /* size of data in byte */

    uint32_t size;

    /* firmware data bytes */

    uint8_t  data[];
};

struct mpu_axes_s {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct mpu_quat_s {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
};

struct mpu_data_mpu_s {
    struct mpu_axes_s accel;
    struct mpu_axes_s gyro;
#ifdef MPU_COMPASS
    struct mpu_axes_s comp;
#endif
};

struct mpu_data_dmp_s {
    struct mpu_quat_s   quat;
    struct mpu_axes_s   accel;
    struct mpu_axes_s   gyro;
};

struct mpu_reg_desc_s {
    const char * desc;
    uint8_t reg;
};

struct mpu_low_s;
struct mpu_low_ops_s {
    /* Low Level access to mpu IC */
    CODE int (*mpu_write)(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                     int size);
    CODE int (*mpu_read )(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                     int size);
    /* Low Level access to AK89xx IC */
    CODE int (*akm_write)(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                     int size);
    CODE int (*akm_read )(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                     int size);
};

/* This structure is the generic form of state structure used by lower mpu
 * access driver.  
 *
 * Normally that timer logic will have its own, custom state structure
 * that is simply cast to struct pwm_lowerhalf_s.  In order to perform such casts,
 * the initial fields of the custom state structure match the initial fields
 * of the following generic PWM state structure.
 */
struct mpu_low_s {
  /* The first field of this state structure must be a pointer to the lowlevel
   * operations structure:
   */
    FAR const struct mpu_low_ops_s *ops;
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* low level function */

#ifdef CONFIG_INVENSENSE_SPI

#define MPU9250_SPI_MAXFREQUENCY 1000000

struct spi_dev_s; /* See nuttx/spi/spi.h */
struct mpu_low_s* mpu_low_spi_init(int devno, int akm_addr, 
                                   FAR struct spi_dev_s* spi);
#endif

#ifdef CONFIG_INVENSENSE_I2C
struct i2c_dev_s; /* See nuttx/i2c/i2c_master.h */
struct mpu_low_s* mpu_low_i2c_init(int devno, int mpu_addr, int akm_addr, 
                                   FAR struct i2c_dev_s* i2c);
#endif

/* Setup function */

struct mpu_inst_s; /* See inv_mpu_base.c */
struct mpu_inst_s* mpu_instantiate(FAR struct mpu_low_s* low);

/* device driver setup function */

int mpu_fileops_init(struct mpu_inst_s* inst,const char *path ,int minor );

/* Reset to default function */

int mpu_reset_default(FAR struct mpu_inst_s* inst);

/* Low Power mode */

int mpu_lp_accel_mode(struct mpu_inst_s* inst, uint8_t rate);
int mpu_lp_motion_interrupt(struct mpu_inst_s* inst,uint16_t thresh, 
                            uint8_t time, uint8_t lpa_freq);

int mpu_get_lpf(struct mpu_inst_s* inst,uint16_t *lpf);
int mpu_set_lpf(struct mpu_inst_s* inst,uint16_t lpf);

/* DMP */

int mpu_set_dmp_state(struct mpu_inst_s* inst,bool enable);
int mpu_get_dmp_state(struct mpu_inst_s* inst,bool *enabled);

/* FSR */

int mpu_get_accel_fsr(  struct mpu_inst_s* inst,uint8_t  *fsr);
int mpu_set_accel_fsr(  struct mpu_inst_s* inst,uint8_t   fsr);

int mpu_get_gyro_fsr(   struct mpu_inst_s* inst,uint16_t *fsr);
int mpu_set_gyro_fsr(   struct mpu_inst_s* inst,uint16_t  fsr);

int mpu_get_compass_fsr(struct mpu_inst_s* inst,uint16_t *fsr);

/* sensibility */

int mpu_get_accel_sensibility(   struct mpu_inst_s* inst, uint16_t *sens);
int mpu_get_gyro_sensibility(    struct mpu_inst_s* inst, float *sens);

/* sample rate */

int mpu_get_sample_rate(struct mpu_inst_s* inst,uint16_t *rate);
int mpu_set_sample_rate(struct mpu_inst_s* inst,uint16_t rate);

int mpu_get_compass_sample_rate(struct mpu_inst_s* inst,uint16_t *rate);
int mpu_set_compass_sample_rate(struct mpu_inst_s* inst,uint16_t rate);

/* power */

int mpu_get_sensors_enable(struct mpu_inst_s* inst,uint8_t * sensors);
int mpu_set_sensors_enable(struct mpu_inst_s* inst,uint8_t   sensors);

/* offset */

int mpu_get_accel_off(  struct mpu_inst_s* inst,struct mpu_axes_s *accel_off);
int mpu_set_accel_off(  struct mpu_inst_s* inst,struct mpu_axes_s *accel_off);

int mpu_get_gyro_off(   struct mpu_inst_s* inst,struct mpu_axes_s *gyro_off);
int mpu_set_gyro_off(   struct mpu_inst_s* inst,struct mpu_axes_s *gyro_off);

/* read only */

int mpu_get_accel_raw(  struct mpu_inst_s* inst,struct mpu_axes_s *data);
int mpu_get_gyro_raw(   struct mpu_inst_s* inst,struct mpu_axes_s *data);
int mpu_get_compass_raw(struct mpu_inst_s* inst,struct mpu_axes_s *data);

int mpu_get_temperature(struct mpu_inst_s* inst, int32_t *temp_degre);

/* interrupt */

int mpu_get_int_status( struct mpu_inst_s* inst, uint8_t *mpu_int_status, 
                       uint8_t *dmp_int_status);
int mpu_set_int_level(  struct mpu_inst_s* inst,bool active_low);
int mpu_set_int_latched(struct mpu_inst_s* inst,bool enable);

/* fifo */

int mpu_set_fifo_config(struct mpu_inst_s* inst,uint8_t sensors);
int mpu_get_fifo_config(struct mpu_inst_s* inst,uint8_t *sensors);

int mpu_read_fifo(struct mpu_inst_s* inst, struct mpu_data_mpu_s *data);

int mpu_fifo_packet_nbr(struct mpu_inst_s* inst);
int mpu_read_fifo_level(struct mpu_inst_s* inst);
int mpu_read_fifo_stream(struct mpu_inst_s* inst,uint8_t *data, int size);

int mpu_reset_fifo(struct mpu_inst_s* inst);

int mpu_write_mem(struct mpu_inst_s* inst,uint16_t mem_addr, uint16_t length,
                  const uint8_t *data);
int mpu_read_mem(struct mpu_inst_s* inst,uint16_t mem_addr, uint16_t length,
                 uint8_t *data);
int mpu_load_firmware(struct mpu_inst_s* inst,uint16_t length, 
                      const uint8_t *firmware, uint16_t start_addr, 
                      uint16_t sample_rate);

int mpu_reg_dump(struct mpu_inst_s* inst);
int mpu_read_reg(struct mpu_inst_s* inst,uint8_t reg, uint8_t *data);

int mpu_set_bypass(struct mpu_inst_s* inst, bool bypass_on);

int mpu_set_dmp_on(struct mpu_inst_s* inst);
int mpu_set_dmp_off(struct mpu_inst_s* inst);

int mpu_calibration(struct mpu_inst_s* inst, struct mpu_data_mpu_s* sensors_off, 
                    int samples_nbr);

int mpu_set_accel_off_safe(struct mpu_inst_s* inst,struct mpu_axes_s *accel_off);

#if 0

SCHED_FIFO

/* TODO Sorry, not implemented */

int mpu_run_self_test(struct mpu_inst_s* inst,long *gyro, long *accel);
int mpu_run_6500_self_test(struct mpu_inst_s* inst,long *gyro, long *accel, 
                           uint8_t debug);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif  /* #ifndef __INCLUDE_NUTTX_INPUT_INV_MPU_H */

