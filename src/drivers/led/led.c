/***********************************************************************
 * @file		: led.c 
 * @brief		: led's driver
 * @author		: huang li long <huanglilongwk@outlook.com>
 * @time		: 2016/09/05
 ***********************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>

#include "chip.h"
#include "stm32f429i-disco.h"

void led_register(void);
int led_main(int argc, char *argv[]);

typedef struct file file_t;

static int led_open(file_t *filep);
static int led_close(file_t *filep);
static ssize_t led_read(file_t *filep, char *buffer, size_t bufferlen);
static ssize_t led_write(file_t *filep, const char *buffer, size_t bufferlen);

static const struct file_operations led_ops = {
	led_open,			/* open */
	led_close,			/* close */
	led_read,			/* read */
	led_write,			/* write */
	0,				 	/* seek */
	0,					/* ioctl */
	0
};

static int led_open(file_t *filep)
{
	//  configure gpio for led1
	stm32_configgpio(GPIO_LED1);
	return 0;
}

static int led_close(file_t *filep)
{
	// nothing
	return 0;
}

static ssize_t led_read(file_t *filep, char *buffer, size_t bufferlen)
{
	// nothing or return led's status 
	uint8_t on = *buffer > 0 ? 1:0;
	stm32_gpiowrite(GPIO_LED1, on);
	return 0;
}

static ssize_t led_write(file_t *filep, const char *buffer, size_t bufferlen)
{
	// turn on/off 
	uint8_t on = *buffer > 0 ? 1:0;
	stm32_gpiowrite(GPIO_LED1, on);
	return 0;
}

void led_register(void)
{
	stm32_configgpio(GPIO_LED1);
	(void)register_driver("/dev/led1", &led_ops, 0444, NULL);
}

int led_main(int argc, char *argv[])
{
	led_register();
	int fd = open("/dev/led1", O_RDONLY);
	//printf("fd is %d\n", fd);

	if (argc < 2) {
		printf("missing command\n");
		return 0;
	}

	if (!strcmp(argv[1], "on")) {
		char on = 1;
		read(fd, &on, sizeof(on));
		return 0;
	}

	if (!strcmp(argv[1], "off")) {
		char on = 0;
		read(fd, &on, sizeof(on));
		return 0;
	}

	printf("invaild command args\n");
	return 0;
}
