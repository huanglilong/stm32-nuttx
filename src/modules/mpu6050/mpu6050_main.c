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
    float buf[6];

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
        int ret = read(fd, (char *)&buf, sizeof(buf)/sizeof(buf[0]));
        if(ret == 6)
        {
            /* printf can't work with float, i don't kown why */
            printf("ax:%8.4f\tay:%8.4f\taz:%8.4f\tgx:%8.4f\tgy:%8.4f\tgz:%8.4f\t\n", 
                   (double)buf[0], (double)buf[1], (double)buf[2], (double)buf[3], (double)buf[4], (double)buf[5]);
        }
        sleep(1);
    }
    return 0;
}
