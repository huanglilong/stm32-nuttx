/****************************************************************************
 * @file    : drivers/mpu6050/mpu_fileops.c
 * @author  : Pierre-noel Bouteville <pnb990@gmail.com>
            : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/06
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "mpu.h"

#if defined(CONFIG_SENSORS_INVENSENSE)
/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/ 

#ifndef CONFIG_INVENSENSE_POLLING_MS
#   define CONFIG_INVENSENSE_POLLING_MS 100
#endif

#define MPU_LOG(...)
//#define MPU_LOG(...) lldbg(__VA_ARGS__)
//#define MPU_LOG(...) syslog(LOG_NOTICE,__VA_ARGS__)

#ifndef BOARD_ENABLES_SENSORS
#   define BOARD_ENABLES_SENSORS MPU_XYZ_GYRO|MPU_XYZ_ACCEL
#endif

#ifndef BOARD_DEFAULT_MPU_HZ
#   define BOARD_DEFAULT_MPU_HZ (100) /* in Hz */
#endif

#ifndef BOARD_DEFAULT_GYRO_FSR
#   define BOARD_DEFAULT_GYRO_FSR (2000) /* in DPS */
#endif

#ifndef BOARD_DEFAULT_ACCEL_FSR
#   define BOARD_DEFAULT_ACCEL_FSR (16) /* in G */
#endif

#ifndef BOARD_ENABLES_DMP_FEATURES
#   define BOARD_ENABLES_DMP_FEATURES ( DMP_FEATURE_6X_LP_QUAT      | \
                                        DMP_FEATURE_SEND_RAW_ACCEL  | \
                                        DMP_FEATURE_SEND_RAW_GYRO   )
#endif

#ifndef __DEFAULT_DMP_HZ
#   define __DEFAULT_DMP_HZ (10) /* in Hz */
#endif

#ifndef __CALIBRATION_SAMPLE_NBR
#   define __CALIBRATION_SAMPLE_NBR (10*__DEFAULT_DMP_HZ) /* 10 second */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef FAR struct file file_t;

struct mpu_dev_s {
    int     open_count;
    struct mpu_inst_s* inst;
#ifdef CONFIG_INVENSENSE_DMP
    struct dmp_s *dmp;
#endif
    int     pkt_in_fifo;
    sem_t   exclsem;
#ifndef CONFIG_DISABLE_POLL
    sem_t   *poll_sem;
#ifndef BOARD_INV_MPU_IRQ
    struct work_s work;
#endif
#endif
};

/****************************************************************************
 * Private Functions prototypes
 ****************************************************************************/

/* Character driver methods */

static int     mpu_open(FAR struct file *filep);
static int     mpu_close(FAR struct file *filep);
static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int     mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     mpu_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

#ifndef CONFIG_DISABLE_POLL
static void mpu_worker(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_mpu_fops =
{
  mpu_open,     /* open */
  mpu_close,    /* close */
  mpu_read,     /* read */
  0,            /* write */
  0,            /* seek */
  mpu_ioctl,    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  mpu_poll,     /* poll */
#endif
  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/******************************************************************************
 * Name: mpu_takesem
 ******************************************************************************/

static int mpu_takesem(FAR sem_t *sem, bool errout)
{
  /* Loop, ignoring interrupts, until we have successfully acquired the semaphore */

  while (sem_wait(sem) != OK)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(get_errno() == EINTR);

      /* When the signal is received, should we errout? Or should we just continue
       * waiting until we have the semaphore?
       */

      if (errout)
        {
          return -EINTR;
        }
    }

  return OK;
}

/******************************************************************************
 * Name: mpu_givensem
 ******************************************************************************/

#define mpu_givesem(sem) (void)sem_post(sem)

/******************************************************************************
 * Name: mpu_get_packet_nbr
 ******************************************************************************/
static inline int mpu_get_packet_nbr(struct mpu_dev_s* dev)
{
    int res;

#ifdef CONFIG_INVENSENSE_DMP
    if ( dev->dmp )
    {
        res = dmp_fifo_packet_nbr(dev->dmp);
    }
    else
#endif
    {
        res = mpu_fifo_packet_nbr(dev->inst);
    }

    return res;
}

/****************************************************************************
 * Name: mpu_set_next_poll
 ****************************************************************************/
#if !defined(CONFIG_DISABLE_POLL) && !defined(BOARD_INV_MPU_IRQ)
static void mpu_set_next_poll(struct mpu_dev_s* dev)
{
    if ( work_queue(HPWORK, 
                    &(dev->work), 
                    mpu_worker,
                    dev, 
                    MSEC2TICK(CONFIG_INVENSENSE_POLLING_MS)
                    ) != OK )
    {
        MPU_LOG("Cannot register worker !\n");
        return;
    }
}
#endif

/****************************************************************************
 * irq handler
 ****************************************************************************/
#ifndef CONFIG_DISABLE_POLL
static void mpu_worker(FAR void *arg)
{
    int res;
    struct mpu_dev_s *dev = (struct mpu_dev_s*)arg;

    res = mpu_takesem(&dev->exclsem, true);
    if ( res < 0 )
        return;

    dev->pkt_in_fifo =  mpu_get_packet_nbr(dev);

    if ( dev->pkt_in_fifo < 0 )
    {
        MPU_LOG("Cannot number of packet in fifo !\n");
    }
    else if ( dev->pkt_in_fifo > 0 )
    {
        MPU_LOG("There is %d packet in fifo\n",dev->pkt_in_fifo);

        MPU_LOG("Data ready !\n");

        /* add event to waiting semaphore */

        if ( dev->poll_sem )
        {
            sem_post( dev->poll_sem );
        }
    }

    mpu_givesem(&dev->exclsem);

#if !defined(BOARD_INV_MPU_IRQ)
    mpu_set_next_poll(dev);
#endif

    return;
}
#endif

/****************************************************************************
 * Name: mpu_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    res = mpu_takesem(&dev->exclsem, true);
    if ( res < 0 )
        return res;

    dev->open_count ++;

    if ( dev->open_count == 1 )
    {
        /* Get/set hardware configuration. Start gyro.
         * Wake up all sensors.
         */

        if ( res == 0 )
            res = mpu_set_sensors_enable(dev->inst, BOARD_ENABLES_SENSORS );

        /* Push both gyro and accel data into the FIFO at default rate. */

        if ( res == 0 )
            res = mpu_set_fifo_config(dev->inst, BOARD_ENABLES_SENSORS );

        if ( res == 0 )
            res = mpu_set_sample_rate(dev->inst, BOARD_DEFAULT_MPU_HZ );

        if ( res == 0 )
            res = mpu_set_accel_fsr(dev->inst,BOARD_DEFAULT_ACCEL_FSR);

        if ( res == 0 )
            res = mpu_set_gyro_fsr(dev->inst,BOARD_DEFAULT_GYRO_FSR);


    #ifdef CONFIG_INVENSENSE_DMP
        if ( dev->dmp == NULL )
        {
            dev->dmp = dmp_init(dev->inst);
        }
    #endif

#ifdef CONFIG_INVENSENSE_DMP
        if ( dev->dmp )
        {
            if ( res == 0 )
                res = dmp_enable_feature(dev->dmp,BOARD_ENABLES_DMP_FEATURES);

            if ( res == 0 )
                res = dmp_set_fifo_rate(dev->dmp,__DEFAULT_DMP_HZ);

            if ( res == 0 )
                res = mpu_set_dmp_on(dev->inst);
        }
#endif
    }

    if ( res < 0 )
        dev->open_count --;

    mpu_givesem(&dev->exclsem);

    return res;
}

/****************************************************************************
 * Name: mpu_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    res = mpu_takesem(&dev->exclsem, true);
    if ( res < 0 )
        return res;

    dev->open_count--;

    DEBUGASSERT(dev->open_count >= 0);

    if ( dev->open_count == 0 )
    {
#ifdef CONFIG_INVENSENSE_DMP
        if ( dev->dmp )
        {
            if ( dmp_enable_feature(dev->dmp,0) < 0 )
                MPU_LOG("Cannot disable dmp feature !\n");

            if ( mpu_set_dmp_off(dev->inst) < 0 )
                MPU_LOG("Cannot turn off dmp !\n");
        }
#endif
        res = mpu_set_sensors_enable(dev->inst,0);

        if ( res >= 0 )
            res = mpu_reset_fifo(dev->inst);
    }

    mpu_givesem(&dev->exclsem);

    return res;
}

/****************************************************************************
 * Name: mpu_poll
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int mpu_poll(file_t * filep, FAR struct pollfd *fds, bool setup)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev    = inode->i_private;

    res = mpu_takesem(&dev->exclsem, true);
    if ( res < 0 )
        return res;

    /* don't read if interrupt have already confirm that some data are ready */

    if( dev->pkt_in_fifo <= 0  )
    {
        dev->pkt_in_fifo = mpu_get_packet_nbr(dev);
    }

    if (setup)
    {

        fds->revents = 0;
        /* This is a request to set up the poll.  Find an available
         * slot for the poll structure reference
         */

        if ( dev->poll_sem != NULL)
        {
            res = -EINVAL;
            goto errout;
        }

        if( dev->pkt_in_fifo > 0  )
        {
            fds->revents |= (fds->events & POLLIN);
        }

        if ( fds->revents == 0 )
        {
            dev->poll_sem = fds->sem ;
        }
        else
        {
            sem_post(fds->sem);
            res = 1;
        }
    }
    else if ( dev->poll_sem == fds->sem )
    {
        dev->poll_sem = NULL;

        if( dev->pkt_in_fifo > 0  )
        {
            fds->revents |= (fds->events & POLLIN);
        }
    }
errout:
    mpu_givesem(&dev->exclsem);
    return res;
}
#endif
/****************************************************************************
 * Name: mpu_read
 *
 * Description:
 *   read fifo
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev    = inode->i_private;

    size_t size = 0;

    res = mpu_takesem(&dev->exclsem, true);
    if ( res < 0 )
        return res;

    if( dev->pkt_in_fifo <= 0  )
    {
        dev->pkt_in_fifo = mpu_get_packet_nbr(dev);
    }

    while ( ( size < len ) && (dev->pkt_in_fifo > 0 ) )
    {
        res = 0;
#ifdef CONFIG_INVENSENSE_DMP
        if ( dev->dmp )
        {
            res = dmp_read_fifo(dev->dmp, (struct mpu_data_dmp_s*)buffer);
            if ( res > 0 )
                size += sizeof(struct mpu_data_dmp_s);
        }
        else
#endif
        {
            res = mpu_read_fifo(dev->inst, (struct mpu_data_mpu_s*)buffer);
            if ( res > 0 )
                size += sizeof(struct mpu_data_mpu_s);
        }

        if ( res <= 0 )
            break;

        dev->pkt_in_fifo --;
    }


    mpu_givesem(&dev->exclsem);


    if ( res < 0 )
        return res;

    return size;
}

/****************************************************************************
 * Name: mpu_ioctl
 *
 * Description:
 *  The invensense MPU ioctl handler
 *
 ****************************************************************************/

static int mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int res = -EINVAL;

    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    switch (cmd)
    {

        case MPU_ENABLE:
            res = mpu_set_sensors_enable(dev->inst,arg);
            if ( res == 0 )
                res = mpu_set_fifo_config(dev->inst,arg);
            break;

        case MPU_RESET_FIFO:
            res = mpu_reset_fifo(dev->inst);
            break;

        case MPU_GET_TEMP:
            res = mpu_get_temperature(dev->inst,(int32_t*)arg);
            break;

        case MPU_SET_GYRO_OFF:
            res = mpu_set_gyro_off(dev->inst,(struct mpu_axes_s*)arg);
            if ( res >= 0 )
                res = mpu_reset_fifo(dev->inst);
            break;

        case MPU_SET_ACCEL_OFF:
            res = mpu_set_accel_off_safe(dev->inst,(struct mpu_axes_s*)arg);
            break;

        case MPU_GET_GYRO_OFF:
            res = mpu_get_gyro_off(dev->inst,(struct mpu_axes_s*)arg);
            break;

        case MPU_GET_ACCEL_OFF:
            res = mpu_get_accel_off(dev->inst,(struct mpu_axes_s*)arg);
            break;

        case MPU_CALIBRATION:
            res = mpu_calibration(dev->inst,(struct mpu_data_mpu_s*)arg,
                                  __CALIBRATION_SAMPLE_NBR);
            break;

        case MPU_DUMP_REG:
            res = mpu_reg_dump(dev->inst);
            break;

        case MPU_FREQUENCY:
            if ( arg < UINT16_MAX )
            {
#ifdef CONFIG_INVENSENSE_DMP
                if ( dev->dmp )
                {
                    res = dmp_set_fifo_rate(dev->dmp,arg);
                }
                else
#endif
                {
                    res = mpu_set_sample_rate(dev->inst,arg);
                }
            }
            break;

    }
    return res;
}


#if BOARD_INV_MPU_IRQ
/****************************************************************************
 * Name: mpu_interrupt
 *
 * Description:
 *  The MPU interrupt handler
 *
 ****************************************************************************/

static void mpu_interrupt(FAR struct mpu_config_s *config, FAR void *arg)
{
  int res;
  FAR struct mpu_dev_s *priv = (FAR struct mpu_dev_s *)arg;

  DEBUGASSERT(priv && priv->config == config);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Yes.. Transfer processing to the worker thread.  Since MPU
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      res = work_queue(HPWORK, &priv->work, mpu_worker, priv, 0);
      if (res != 0)
        {
          snlldbg("Failed to queue work: %d\n", res);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_fileops_init
 *
 * Description:
 *   regiter invensense mpu ic.
 *
 * Input Parameters:
 *   path      - path to register this device
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mpu_fileops_init(struct mpu_inst_s* inst,const char *path ,int minor )
{
    int res = 0;
    FAR struct mpu_dev_s *dev;

    /* Allocate the MPU driver instance */

    dev = (FAR struct mpu_dev_s *)kmm_zalloc(sizeof(struct mpu_dev_s));
    if (!dev)
    {
        //sndbg("Failed to allocate the device structure!\n");
        return -ENOMEM;
    }

    DEBUGASSERT(dev);

    /* Initialize the device state structure */

    memset(dev,0,sizeof(*dev));
    //dev->poll_sem = NULL; /* already done by memset */
    sem_init(&dev->exclsem, 0, 1);
    dev->inst = inst;



    /* Register the character driver */

    res = register_driver(path, &g_mpu_fops, 0666, dev);

    if (res < 0)
    {
        //sndbg("ERROR: Failed to register driver %s: %d\n", devname, res);
        return res;
    }

#ifndef CONFIG_DISABLE_POLL
    mpu_set_next_poll(dev);
#endif

    return res;
}

#endif /* CONFIG_SENSORS_INVENSENSE */
