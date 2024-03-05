#ifndef __AS5600_DEV_H__
#define __AS5600_DEV_H__

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

static esp_err_t as5600_dev_init(gpio_num_t SCL_IO,gpio_num_t );

#ifdef __cplusplus
}
#endif
#endif