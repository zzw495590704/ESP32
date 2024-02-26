#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "sensor_app.h"
#include "uart_dev.h"
static const char *TAG = "sensor_app";
TaskHandle_t sensor_app_task_Handle;
uint8_t senser_cmd[][5] = {
    {0xa3,0x00,0xa2,0xa4,0xa5}
};
int sensor_app_read_weight(void){
    uint8_t data[12];
    uart_dev_send(senser_cmd[0],5);
    uint16_t buf_len = uart_dev_recive(data,10,500);
    ESP_LOG_BUFFER_HEX("setting_sensor_return",data,buf_len);
    int weight = data[4]<<16 | data[5]<<8 | data[6];
    if (data[3]==0x01){
        weight *= -1; 
    }
    ESP_LOGI(TAG,"weight:%d",weight);
    return weight;
}

 void sensor_app_task(void *arg){
    while (1){
        sensor_app_read_weight();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(sensor_app_task_Handle);
 }

void sensor_app_task_creat(void){
    xTaskCreate(sensor_app_task, "sensor_app_task", 1024*3, NULL, configMAX_PRIORITIES-1, &sensor_app_task_Handle);
}

void sensor_app_init(){
    uart_dev_init();
    sensor_app_task_creat();
}
