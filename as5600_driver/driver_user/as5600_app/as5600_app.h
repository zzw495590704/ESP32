#ifndef __AS5600_APP_H__
#define __AS5600_APP_H__

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct 
{
    uint16_t value;
    uint16_t init_value;
    uint16_t last_value;
    uint8_t direction;
    int circle;
    float angle;
    float totol_angle;
}as5600_data;

void as5600_app_init();

#ifdef __cplusplus
}
#endif
#endif