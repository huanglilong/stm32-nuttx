/****************************************************************************
 * drivers/mpu6050/mpu6050.c
 * Character driver for the Invensense MPU6050 IMU Sensor
 *
 *   Copyright (C) 2016 huang li long. All rights reserved.
 *   Author: huang li long <huanglilongwk@outlook.com>
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
/* convert unused-function compile error to warning */
#pragma GCC diagnostic warning "-Wunused-function"

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip.h"
#include "stm32f429i-disco.h"
#include "mpu6050.h"

#if defined(CONFIG_I2C) && defined(CONFIG_MPU6050)

/***************** MPU6050 REGISTERS *******************/
#define MPU6050_DEVICE_ADDRESS  0xD0		
#define MPU6050_I2C_FREQ_MAX    400000

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6
/***************** MPU6050 REGISTERS *******************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
struct mpu6050_dev_s
{
	FAR struct i2c_master_s *i2c;
	uint8_t addr;                 	/* MPU6050 I2C address */
	int freq;                     	/* MPU6050 Frequency <= 4KHz */
	float acc_x;				  	/* accelerometer x axis */
	float acc_y;				  	/* accelerometer y axis */
	float acc_z;				  	/* accelerometer z axis */
	// float acc_x_offset;				/* accelerometer x axis */
	// float acc_y_offset;				/* accelerometer y axis */
	// float acc_z_offset;				/* accelerometer z axis */

	float gy_x;				 		/* gyroscope x axis */
	float gy_y;				 		/* gyroscope y axis */
	float gy_z;				 		/* gyroscope z axis */
	// float gy_x_offset;				/* gyroscope x offset */
	// float gy_y_offset;				/* gyroscope y offset */
	// float gy_z_offset;				/* gyroscope z offset */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static uint8_t  mpu6050_getreg8 (FAR struct mpu6050_dev_s *priv, uint8_t regaddr);
static uint16_t mpu6050_getreg16(FAR struct mpu6050_dev_s *priv, uint8_t regaddr);
static void     mpu6050_putreg8 (FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t regval);

static void 	mpu6050_set_clock_source(FAR struct mpu6050_dev_s *priv, uint8_t source);
static void 	mpu6050_set_full_scale_gyro_range(FAR struct mpu6050_dev_s *priv, uint8_t range);
static void 	mpu6050_set_full_scale_accel_range(FAR struct mpu6050_dev_s *priv, uint8_t range);
static void 	mpu6050_set_sleep_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled);
static uint8_t 	mpu6050_get_device_id(FAR struct mpu6050_dev_s *priv);
static void 	mpu6050_set_i2c_master_mode_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled);
static void 	mpu6050_set_i2c_bypass_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled);
static void 	mpu6050_initialize(FAR struct mpu6050_dev_s *priv);
static uint8_t 	mpu6050_is_rdy(void);
static void 	mpu6050_get_motion(FAR struct mpu6050_dev_s *priv);
static void 	mpu6050_init_gyro_Offset(void);
static void 	mpu6050_interrupt_config(FAR struct mpu6050_dev_s *priv);
static void     mpu6050_write_bits(FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t bit_start, uint8_t len, uint8_t data);
static void 	mpu6050_write_bit(FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t bit_num, uint8_t data);

/* Character driver methods */
static int     mpu6050_open (FAR struct file *filep);
static int 	   mpu6050_close(FAR struct file *filep);
static ssize_t mpu6050_read (FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t mpu6050_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct file_operations g_mpu6050fops =
{
  mpu6050_open,                  /* open */
  mpu6050_close,                 /* close */
  mpu6050_read,                  /* read */
  mpu6050_write,                 /* write */
  0,                            /* seek */
  0,                            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                            /* poll */
#endif
  0                             /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu6050_getreg8
 *
 * Description:
 *   Read from an 8-bit MPU6050 register
 *
 ****************************************************************************/
static uint8_t mpu6050_getreg8 (FAR struct mpu6050_dev_s *priv, uint8_t regaddr)
{
	struct i2c_config_s config;
	uint8_t regval = 0;
	int ret;

	/* set up i2c configuration */
	config.frequency  	= priv->freq;
	config.address		= priv->addr;
	config.addrlen		= 7;

	/* write the register address */
	ret = i2c_write(priv->i2c, &config, &regaddr, 1);
	if(ret < 0)
	{
		snerr("ERROR: i2c_write failed: %d\n", ret);
		return ret;
	}

	/* read the register value */
	ret = i2c_read(priv->i2c, &config, &regval, 1);
	if(ret < 0)
	{
		snerr("ERROR: i2c_read failed: %d\n", ret);
		return ret;
	}

	return regval;
}

/****************************************************************************
 * Name: mpu6050_getreg16
 *
 * Description:
 *   Read two 8-bit from a MPU6050 register
 *
 ****************************************************************************/
static uint16_t mpu6050_getreg16(FAR struct mpu6050_dev_s *priv, uint8_t regaddr)
{
	struct i2c_config_s config;
	uint16_t msb, lsb;
	uint16_t regval = 0;
	int ret;

	/* set up i2c configuration */
	config.frequency	= priv->freq;
	config.address		= priv->addr;
	config.addrlen		= 7;

	/* write the register address */
	ret = i2c_write(priv->i2c, &config, &regaddr, 1);
	if(ret < 0)
	{
		snerr("ERROR: i2c_write failed: %d\n", ret);
		return ret;	
	}

	/* read the regiter value */
	ret  = i2c_read(priv->i2c, &config, (uint8_t *)&regval, 2);
	if(ret < 0)
	{
		snerr("ERROR: i2c_read failed: %d\n", ret);
		return ret;
	}

	/* MSB and LSB are inverted */
	msb = (regval & 0xFF);
	lsb = (regval & 0xFF00) >> 8;
	regval = (msb << 8) | lsb;

	return regval;

}

/****************************************************************************
 * Name: mpu6050_putreg8
 *
 * Description:
 *   Write to an 8-bit MPU6050 register
 *
 ****************************************************************************/
 static void mpu6050_putreg8 (FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t regval)
 {
	 struct i2c_config_s config;
	 uint8_t data[2];
	 int ret;

	 /* set up i2c configuration */
	 config.frequency	= priv->freq;
	 config.address		= priv->addr;
	 config.addrlen		= 7;

	 data[0] = regaddr;
	 data[1] = regval;

	 /* write the register address and value */
	 ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 2);
	 if(ret < 0)
	 {
		snerr("ERROR: i2c_write failed: %d\n", ret);
		return;
	 }

	 return;
 }

static void mpu6050_write_bits(FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t bit_start, uint8_t len, uint8_t data)
{
    uint8_t regval, mask;
    regval = mpu6050_getreg8(priv, regaddr);
    mask = (0xFF << (bit_start + 1)) | 0xFF >> ((8 - bit_start) + len - 1);
    data <<= (8 - len);
    data >>= (7 - bit_start);
    regval &= mask;
    regval |= data;
	mpu6050_putreg8(priv, regaddr, regval);
}

static void mpu6050_write_bit(FAR struct mpu6050_dev_s *priv, uint8_t regaddr, uint8_t bit_num, uint8_t data)
{
	uint8_t regval;
	regval = mpu6050_getreg8(priv, regaddr);
	regval = (data != 0) ? (regval | (1 << bit_num)) : (regval & ~(1 << bit_num));
	mpu6050_putreg8(priv, regaddr, regval);
}

/*
 * Name: mpu6050_get_device_id
 *
 * Description:
 * 	Return mpu6050's i2c address, that should be 0x68 or 0x69
 */
static uint8_t mpu6050_get_device_id(FAR struct mpu6050_dev_s *priv)                  
{
	return mpu6050_getreg8(priv, MPU6050_RA_WHO_AM_I);
}
/*
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
static void mpu6050_set_clock_source(FAR struct mpu6050_dev_s *priv, uint8_t data)
{
	uint8_t regaddr 	= MPU6050_RA_PWR_MGMT_1;
	uint8_t bit_start 	= MPU6050_PWR1_CLKSEL_BIT;
	uint8_t len 		= MPU6050_PWR1_CLKSEL_LENGTH;
	mpu6050_write_bits(priv, regaddr, bit_start, len, data);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
static void mpu6050_set_full_scale_gyro_range(FAR struct mpu6050_dev_s *priv, uint8_t range)   
{
    uint8_t regaddr 	= MPU6050_RA_GYRO_CONFIG;
	uint8_t bit_start 	= MPU6050_GCONFIG_FS_SEL_BIT;
	uint8_t len 		= MPU6050_GCONFIG_FS_SEL_LENGTH;
	mpu6050_write_bits(priv, regaddr, bit_start, len, range);
}

static void mpu6050_set_full_scale_accel_range(FAR struct mpu6050_dev_s *priv, uint8_t range)  
{
	uint8_t regaddr 	= MPU6050_RA_ACCEL_CONFIG;
	uint8_t bit_start 	= MPU6050_ACONFIG_AFS_SEL_BIT;
	uint8_t len 		= MPU6050_ACONFIG_AFS_SEL_LENGTH;
	mpu6050_write_bits(priv, regaddr, bit_start, len, range);
}


static void mpu6050_set_sleep_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled)      
{
	uint8_t regaddr 	= MPU6050_RA_PWR_MGMT_1;
	uint8_t bit_num 	= MPU6050_PWR1_SLEEP_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, enabled);
}

static void mpu6050_set_i2c_master_mode_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled)    
{
	uint8_t regaddr 	= MPU6050_RA_USER_CTRL;
	uint8_t bit_num 	= MPU6050_USERCTRL_I2C_MST_EN_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, enabled);
}

static void mpu6050_set_i2c_bypass_enabled(FAR struct mpu6050_dev_s *priv, uint8_t enabled)       
{
	uint8_t regaddr 	= MPU6050_RA_INT_PIN_CFG;
	uint8_t bit_num 	= MPU6050_INTCFG_I2C_BYPASS_EN_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, enabled);
}

static void mpu6050_interrupt_config(FAR struct mpu6050_dev_s *priv)
{
	uint8_t regaddr 	= MPU6050_RA_INT_PIN_CFG;
	uint8_t bit_num 	= MPU6050_INTCFG_INT_LEVEL_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, 0);

	regaddr 			= MPU6050_RA_INT_PIN_CFG;
	bit_num 			= MPU6050_INTCFG_INT_OPEN_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, 0);

	regaddr 			= MPU6050_RA_INT_PIN_CFG;
	bit_num 			= MPU6050_INTCFG_LATCH_INT_EN_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, 1);

	regaddr 			= MPU6050_RA_INT_PIN_CFG;
	bit_num 			= MPU6050_INTCFG_INT_RD_CLEAR_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, 1);

	regaddr 			= MPU6050_RA_INT_ENABLE;
	bit_num 			= MPU6050_INTERRUPT_DATA_RDY_BIT;
	mpu6050_write_bit(priv, regaddr, bit_num, 1);
}

static void mpu6050_initialize(FAR struct mpu6050_dev_s *priv)
{
     /* mpu6050 interrupt digital input */
	 stm32_configgpio(GPIO_MPU6050_INT);
	 mpu6050_set_clock_source(priv, MPU6050_CLOCK_PLL_XGYRO);
     mpu6050_set_full_scale_gyro_range(priv, MPU6050_GYRO_FS_1000);          // +- 1000dps
     mpu6050_set_full_scale_accel_range(priv, MPU6050_ACCEL_FS_2);	          // +/- 2g
     mpu6050_set_sleep_enabled(priv ,0);
     mpu6050_set_i2c_master_mode_enabled(priv, 0);
     mpu6050_interrupt_config(priv);
}

static uint8_t mpu6050_is_rdy(void)
{
	if(stm32_gpioread(GPIO_MPU6050_INT) == 1)
	{
		return 1;		//mpu6050 data is ready
	}
	else
	{
		return 0;
	}
}

static void mpu6050_get_motion(FAR struct mpu6050_dev_s *priv)
{
	uint8_t buffer[12];
	int16_t temp[6];

	/* get accelerometer value */
	buffer[0] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_XOUT_H);
	buffer[1] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_XOUT_L);
	buffer[2] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_YOUT_H);
	buffer[3] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_YOUT_L);
	buffer[4] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_ZOUT_H);
	buffer[5] = mpu6050_getreg8(priv, MPU6050_RA_ACCEL_ZOUT_L);

	/* get gyroscope value */
	buffer[6]  = mpu6050_getreg8(priv, MPU6050_RA_GYRO_XOUT_H);
	buffer[7]  = mpu6050_getreg8(priv, MPU6050_RA_GYRO_XOUT_L);
	buffer[8]  = mpu6050_getreg8(priv, MPU6050_RA_GYRO_YOUT_H);
	buffer[9]  = mpu6050_getreg8(priv, MPU6050_RA_GYRO_YOUT_L);
	buffer[10] = mpu6050_getreg8(priv, MPU6050_RA_GYRO_ZOUT_H);
	buffer[11] = mpu6050_getreg8(priv, MPU6050_RA_GYRO_ZOUT_L);

	/* calculate acc and gyro's value */
	temp[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
	temp[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
	temp[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
	temp[3] = (((int16_t)buffer[6]) << 8) | buffer[7];
	temp[4] = (((int16_t)buffer[8]) << 8) | buffer[9];
	temp[5] = (((int16_t)buffer[10]) << 8)| buffer[11];

	/* acc range is +/- 2g */
	priv->acc_x = 4.0f * temp[0] / 65535;
	priv->acc_y = 4.0f * temp[1] / 65535;
	priv->acc_z = 4.0f * temp[2] / 65535;

	/* gyro range is +/- 1000dps */
	priv->gy_x = 2000.0f * temp[3] / 65535;
	priv->gy_y = 2000.0f * temp[4] / 65535;
	priv->gy_z = 2000.0f * temp[5] / 65535;
}

static void mpu6050_init_gyro_Offset(void)
{

}

/****************************************************************************
 * Name: mpu6050_open
 *
 * Description:
 *   This function is called whenever the MPU6050 device is opened.
 *
 ****************************************************************************/
static int mpu6050_open (FAR struct file *filep)
{
	/* init MPU6050 */
	FAR struct inode			*inode = filep->f_inode;
	FAR struct mpu6050_dev_s 	*priv  = inode->i_private;

	if(mpu6050_get_device_id(priv) == MPU6050_DEFAULT_ADDRESS)
	{
		stm32_configgpio(GPIO_MPU6050_INT);
		mpu6050_initialize(priv);  // init mpu6050
		return OK; // ok
	}
	else
	{
		return -1;
	}
}

/****************************************************************************
 * Name: mpu6050_close
 *
 * Description:
 *   This routine is called when the MPU6050 device is closed.
 *
 ****************************************************************************/
static int mpu6050_close(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: mpu6050_read
 ****************************************************************************/
static ssize_t mpu6050_read (FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	FAR struct inode			*inode = filep->f_inode;
	FAR struct mpu6050_dev_s 	*priv  = inode->i_private;

	if(mpu6050_is_rdy())	// if data is ready
	{
		mpu6050_get_motion(priv);

		if(buflen == 6)
		{
			float *buf = (float *)buffer;
			buf[0] = priv->acc_x;
			buf[1] = priv->acc_y;
			buf[2] = priv->acc_z;
			buf[3] = priv->gy_x;
			buf[4] = priv->gy_y;
			buf[5] = priv->gy_z;
			return 6;
		}		
	}
	return 0;
}

/****************************************************************************
 * Name: mpu6050_write
 ****************************************************************************/
static ssize_t mpu6050_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu6050_register
 *
 * Description:
 *   Register the MPU6050 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mpu6050"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MPU6050
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
 int mpu6050_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
 {
	 FAR struct mpu6050_dev_s *priv;
	 int ret;

	 /* initialize the MPU6050 device structure */
	 priv = (FAR struct mpu6050_dev_s *)kmm_malloc(sizeof(struct mpu6050_dev_s));
	 if(!priv)
	 {
		snerr("ERROR: Failed to allocate instance\n");
		return -ENOMEM;
	 }

	 priv->i2c 		= i2c;
	 priv->addr 	= MPU6050_ADDRESS_AD0_LOW;//MPU6050_DEVICE_ADDRESS;
	 priv->freq		= MPU6050_I2C_FREQ_MAX;

	 /* register the mpu6050 charater driver */
	 ret = register_driver(devpath, &g_mpu6050fops, 0666, priv);
	 if(ret < 0)
	 {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(priv);
	 }

	 sninfo("MPU6050 driver loaded successfully!\n");
	 return ret;
 }
#endif /* CONFIG_I2C && CONFIG_MPU6050 */
