/****************************************************************************
 * @file    : drivers/mpu6050/mpu6500_reg.c
 * @author  : Pierre-noel Bouteville <pnb990@gmail.com>
            : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/06
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "mpu.h"


/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#if ( ( defined CONFIG_SENSOR_MPU6050 ) || \
      ( defined CONFIG_SENSOR_MPU9150 ) )

/* Configuration **************************************************************/

#include "mpu6050_reg.h"

/* Debug **********************************************************************/

#  define regdbg  dbg

/* BIT MASK *******************************************************************/


/*******************************************************************************
 * Private Types
 ******************************************************************************/

const struct mpu_reg_desc_s mpu6050_reg_desc[] =
{
    {"INV_MPU_SELFTEST_X",      INV_MPU_SELFTEST_X      },
    {"INV_MPU_SELFTEST_Y",      INV_MPU_SELFTEST_Y      },
    {"INV_MPU_SELFTEST_Z",      INV_MPU_SELFTEST_Z      },
    {"INV_MPU_SELFTEST_A",      INV_MPU_SELFTEST_A      },

    {"INV_MPU_WHO_AM_I",        INV_MPU_WHO_AM_I        },
    {"INV_MPU_RATE_DIV",        INV_MPU_RATE_DIV        },
    {"INV_MPU_LPF",             INV_MPU_LPF             },
    {"INV_MPU_PROD_ID",         INV_MPU_PROD_ID         },
    {"INV_MPU_USER_CTRL",       INV_MPU_USER_CTRL       },
    {"INV_MPU_FIFO_EN",         INV_MPU_FIFO_EN         },
    {"INV_MPU_GYRO_CFG",        INV_MPU_GYRO_CFG        },
    {"INV_MPU_ACCEL_CFG",       INV_MPU_ACCEL_CFG       },
    {"INV_MPU_MOTION_THR",      INV_MPU_MOTION_THR      },
    {"INV_MPU_MOTION_DUR",      INV_MPU_MOTION_DUR      },
    {"INV_MPU_FIFO_COUNT_H",    INV_MPU_FIFO_COUNT_H    },
    {"INV_MPU_FIFO_R_W",        INV_MPU_FIFO_R_W        },
    {"INV_MPU_RAW_GYRO",        INV_MPU_RAW_GYRO        },
    {"INV_MPU_RAW_ACCEL",       INV_MPU_RAW_ACCEL       },
    {"INV_MPU_TEMP",            INV_MPU_TEMP            },
    {"INV_MPU_INT_ENABLE",      INV_MPU_INT_ENABLE      },
    {"INV_MPU_DMP_INT_STATUS",  INV_MPU_DMP_INT_STATUS  },
    {"INV_MPU_INT_STATUS",      INV_MPU_INT_STATUS      },
    {"INV_MPU_PWR_MGMT_1",      INV_MPU_PWR_MGMT_1      },
    {"INV_MPU_PWR_MGMT_2",      INV_MPU_PWR_MGMT_2      },
    {"INV_MPU_INT_PIN_CFG",     INV_MPU_INT_PIN_CFG     },
    {"INV_MPU_MEM_R_W",         INV_MPU_MEM_R_W         },
    {"INV_MPU_ACCEL_OFFS",      INV_MPU_ACCEL_OFFS      },
    {"INV_MPU_I2C_MST",         INV_MPU_I2C_MST         },
    {"INV_MPU_BANK_SEL",        INV_MPU_BANK_SEL        },
    {"INV_MPU_MEM_START_ADDR",  INV_MPU_MEM_START_ADDR  },
    {"INV_MPU_PRGM_START_H",    INV_MPU_PRGM_START_H    },

    /* Gyroscope offset */

    {"INV_MPU_XG_OFFSET_H",     INV_MPU_XG_OFFSET_H     },
    {"INV_MPU_XG_OFFSET_L",     INV_MPU_XG_OFFSET_L     },
    {"INV_MPU_YG_OFFSET_H",     INV_MPU_YG_OFFSET_H     },
    {"INV_MPU_YG_OFFSET_L",     INV_MPU_YG_OFFSET_L     },
    {"INV_MPU_ZG_OFFSET_H",     INV_MPU_ZG_OFFSET_H     },
    {"INV_MPU_ZG_OFFSET_L",     INV_MPU_ZG_OFFSET_L     },

    /* Accelerometer offset */

    {"INV_MPU_XA_OFFSET_H",     INV_MPU_XA_OFFSET_H     },
    {"INV_MPU_XA_OFFSET_L",     INV_MPU_XA_OFFSET_L     },
    {"INV_MPU_YA_OFFSET_H",     INV_MPU_YA_OFFSET_H     },
    {"INV_MPU_YA_OFFSET_L",     INV_MPU_YA_OFFSET_L     },
    {"INV_MPU_ZA_OFFSET_H",     INV_MPU_ZA_OFFSET_H     },
    {"INV_MPU_ZA_OFFSET_L",     INV_MPU_ZA_OFFSET_L     },
    {"",                        0x00                    }
};


#endif /* ( defined CONFIG_SENSOR_MPU6050)||(defined CONFIG_SENSOR_MPU9150))*/
