/************************************************************************************
 * nuttx-configs/stm32f429discovery/src/stm32_mpu6050.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

//#include <nuttx/spi/spi.h>
//#include <mpu6050.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f429i-disco.h"

#if defined(CONFIG_I2C) && defined(CONFIG_MPU6050)

#define MPU6050_I2C_PORTNO 1
int mpu6050_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

/************************************************************************************
 * Name: stm32_mpu6050_initialize
 *
 * Description:
 *   Initialize and register the MPU6050 Pressure Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mpu6050"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int stm32_mpu6050_initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing MPU6050!\n");

  /* Initialize I2C */
  i2c = stm32_i2cbus_initialize(MPU6050_I2C_PORTNO);

  if (!i2c)
  {
	  return -ENODEV;
  }

  /* Then register the barometer sensor */
  ret = mpu6050_register(devpath, i2c);
  if (ret < 0)
  {
	  snerr("ERROR: Error registering MPU6050\n");
  }

  return ret;
}
#endif
