/****************************************************************************
 * @file    : drivers/mpu6050/mpu_i2c.c
 * @author  : Pierre-noel Bouteville <pnb990@gmail.com>
            : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/06
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/i2c/i2c_master.h>
#include "mpu.h"

#if defined(CONFIG_INVENSENSE_I2C)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu_i2c_low_s
{
    const struct mpu_low_ops_s *ops;
    int mpu_addr;
    int akm_addr;
    struct i2c_master_s* i2c;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mpu_i2c_write(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf,
                         int size);
static int mpu_i2c_read( FAR struct mpu_low_s* low, int reg_off, uint8_t *buf,
                         int size);
static int akm_i2c_write(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf,
                         int size);
static int akm_i2c_read( FAR struct mpu_low_s* low, int reg_off, uint8_t *buf,
                         int size);

/****************************************************************************
 * Private data
 ****************************************************************************/

static const struct mpu_low_ops_s g_mpu_i2c_low_ops = 
{
    .mpu_write = mpu_i2c_write,
    .mpu_read  = mpu_i2c_read,
    .akm_write = akm_i2c_write,
    .akm_read  = akm_i2c_read
};

struct mpu_i2c_low_s g_mpu_i2c;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mpu_i2c_trans(FAR struct mpu_i2c_low_s* i2c_low, int i2c_addr, bool read, 
                         uint8_t reg_off, uint8_t *buf, int size)
{
  struct i2c_msg_s msg[2];
  int ret;

  /* Setup 8-bit MPU address write message */

  msg[0].addr   = i2c_addr;             /* 7-bit address                */
  msg[0].flags  = 0;                    /* Write transaction            */
  msg[0].buffer = &reg_off;             /* Transfer from this address   */
  msg[0].length = 1;                    /* Send one byte address        */

  /* Set up the data written message */

  msg[1].addr   = i2c_addr;             /* 7-bit address                */
  if (read)
      msg[1].flags  = I2C_M_READ;       /* Read transaction             */
  else
      msg[1].flags  = I2C_M_NORESTART;  /* write transaction no restart */
  msg[1].buffer = buf;                  /* Transfer to this address     */
  msg[1].length = size;                 /* Receive/Send data buffer     */

  /* Perform the transfer */

  ret = I2C_TRANSFER(i2c_low->i2c, msg, 2);
  if (ret < 0)
    {
      //sndbg("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return 0;

}

/* MPU Write access */

static int mpu_i2c_write(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                         int size)
{
  struct mpu_i2c_low_s* priv = (struct mpu_i2c_low_s*)low;

  return mpu_i2c_trans(priv,priv->mpu_addr,false,reg_off,buf,size);
}

/* MPU read access */

static int mpu_i2c_read(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                        int size)
{
  struct mpu_i2c_low_s* priv = (struct mpu_i2c_low_s*)low;

  return mpu_i2c_trans(priv,priv->mpu_addr,true,reg_off,buf,size);
}

/* AKM Write access */

static int akm_i2c_write(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                         int size)
{
  struct mpu_i2c_low_s* priv = (struct mpu_i2c_low_s*)low;

  return mpu_i2c_trans(priv,priv->akm_addr,false,reg_off,buf,size);
}

/* AKM read access */

static int akm_i2c_read(FAR struct mpu_low_s* low, int reg_off, uint8_t *buf, 
                        int size)
{
  struct mpu_i2c_low_s* priv = (struct mpu_i2c_low_s*)low;

  return mpu_i2c_trans(priv,priv->akm_addr,true,reg_off,buf,size);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
struct mpu_low_s* mpu_low_i2c_init(int devno, int mpu_addr, int akm_addr, 
                                   FAR struct i2c_master_s* i2c)
{
    struct mpu_i2c_low_s* mpu_i2c;

    /* only one device supported */

    if ( devno != 0 )
        return NULL;

    mpu_i2c = &g_mpu_i2c;

    mpu_i2c->ops = &g_mpu_i2c_low_ops;
    mpu_i2c->mpu_addr = mpu_addr;
    mpu_i2c->akm_addr = akm_addr;
    mpu_i2c->i2c = i2c;

    return (struct mpu_low_s*)mpu_i2c;

}

#endif /* CONFIG_INVENSENSE_I2C */
