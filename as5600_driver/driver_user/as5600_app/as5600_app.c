/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "as5600_dev.h"
#include "as5600_app.h"

static const char *TAG = "as5600_app";
TaskHandle_t as5600_app_task_Handle;
#define I2C_MASTER_SCL1_IO           (GPIO_NUM_6)      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA1_IO           (GPIO_NUM_5)      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define AS600_SENSOR_ADDR                 0x36        
#define AS5600_ANGLE_H           0x0e      
#define AS5600_ANGLE_L           0x0f
#define AS5600_MAX_VALUE         4096



static int as5600_circle=0;
uint16_t as5600_value,as5600_value_last;
int as5600_dir,as5600_diff;
float as5600_angle,as5600_total_angle;

uint16_t as5600_get_value(){
    uint16_t result = 0;
    uint8_t data[2];
    as5600_dev_read(I2C_MASTER_NUM,AS5600_ANGLE_H, &data[0], 1);
    as5600_dev_read(I2C_MASTER_NUM,AS5600_ANGLE_L, &data[1], 1);
    // ESP_LOG_BUFFER_HEX(TAG,data,2);
    result=(uint16_t)(data[0]<<8|data[1]); //一共就11位 注意
    return result;
}
float as5600_get_angle(uint16_t value){
    float angle;
    angle=((int) value & 0b0000111111111111)*360.0/4096.0;
    return angle;
}
int as5600_get_dir(int diff){
    int dir;
    //turn positve over 1 circle
    if (diff<(-1*AS5600_MAX_VALUE/2)){
        dir = AS5600_TURN_N_OVER;
    }
    //turn negative over 1 circle
    else if (diff>(AS5600_MAX_VALUE/2))
    {
        dir = AS5600_TURN_P_OVER;
    }
    else if (diff>0){
        dir = AS5600_TURN_N;
    }
    else if (diff<0){
        dir = AS5600_TURN_P;
    }
    //no turn
    else{
        dir = AS5600_STOP;
    }
    
    return dir;
}
void as5600_get_circle(int dir){
    switch (dir)
    {
    case AS5600_TURN_N_OVER:
        as5600_circle += 1;
        /* code */
        break;
    case AS5600_TURN_P_OVER:
        as5600_circle -= 1;
        break;
    default:
        break;
    }
}
float as5600_get_total_angle(uint16_t value,int circle){
    float angle = as5600_get_angle(value);
    return (float)circle*360+angle;
}

void as5600_app_task(void *arg){
    uint8_t data[2];
    as5600_value_last = as5600_get_value(&data);
    while (1){
        as5600_value = as5600_get_value(&data);
        as5600_angle = as5600_get_angle(as5600_value);
        as5600_diff = as5600_value - as5600_value_last;
        as5600_dir = as5600_get_dir(as5600_diff);
        as5600_get_circle(as5600_dir);
        as5600_total_angle = as5600_get_total_angle(as5600_value,as5600_circle);
        as5600_value_last = as5600_value;
        ESP_LOGI(TAG, "diff:%d angle:%f dir:%d circle:%d total:%.2f",as5600_diff,as5600_angle,as5600_dir,as5600_circle,as5600_total_angle);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void as5600_app_task_creat(void){
    xTaskCreate(as5600_app_task, "as5600_app_task", 1024*3, NULL, configMAX_PRIORITIES-1, &as5600_app_task_Handle);
}

void as5600_app_init(){
    
    ESP_ERROR_CHECK(as5600_dev_init(I2C_MASTER_SDA1_IO,I2C_MASTER_SCL1_IO,I2C_MASTER_NUM));
    ESP_LOGI(TAG, "AS5600 I2C initialized successfully");
    as5600_app_task_creat(); 
}