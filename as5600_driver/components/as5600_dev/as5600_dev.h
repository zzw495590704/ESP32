#ifndef __SENSOR_APP_H
#define __SENSOR_APP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stdint.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2c.h"

esp_err_t as5600_dev_init(int SDA_IO,int SCL_IO,i2c_port_t i2c_master_port);
esp_err_t as5600_dev_read(i2c_port_t i2c_master_port, uint8_t reg_addr, uint8_t *data, size_t len);
#endif

