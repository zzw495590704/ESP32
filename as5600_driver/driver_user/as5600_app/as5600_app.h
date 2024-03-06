#ifndef __AS5600_APP_H__
#define __AS5600_APP_H__

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

#define TXD_PIN (GPIO_NUM_6)
#define RXD_PIN (GPIO_NUM_7)

enum AS5600_DIR_ENUM{
    AS5600_STOP = 0,
    AS5600_TURN_N = 1,
    AS5600_TURN_P = 2,
    AS5600_TURN_N_OVER = 3,
    AS5600_TURN_P_OVER = 4,
};

void as5600_app_init();

#ifdef __cplusplus
}
#endif
#endif