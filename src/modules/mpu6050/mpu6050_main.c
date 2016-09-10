/****************************************************************************
 * @file    : src/module/mpu6050_main.c
 * @author  : huang li long <huanglilongwk@outlook.com>
 * @time    : 2016/09/10
 * @brief   : test mpu6050 device
 */
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
#include <nuttx/fs/fs.h>

#include "chip.h"
#include "stm32f429i-disco.h"

int mpu6050_main(int argc, char *argv[]);
int mpu6050_main(int argc, char *argv[])
{
    int16_t buf[6];

    /* open mpu6050 device */
    int fd = open("/dev/mpu6050", O_RDONLY);
    if(fd < 0)
    {
        printf("Warning: mpu6050 open failed!\n");
        return -1;
    }
    /* poll, wait for data ready */
    while(1)
    {
        int ret = read(fd, (char *)&buf, sizeof(buf));
        if(ret == 6)
        {
            /* printf can't work with float, i don't kown why */
            printf("ax:%4d ay:%4d az:%4d gx:%4d gy:%4d gz:%4d \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        }
        sleep(2);
    }
    return 0;
}