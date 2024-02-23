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

TaskHandle_t sensor_app_task_Handle;
void sensor_app_read_weight(void){
    uint8_t data[12];
    uint8_t set_param[5];
    uart_dev_send(set_param,9);
    // uint16_t buf_len = uart_dev_recive(data,5,500);
    // ESP_LOG_BUFFER_HEX("setting_sensor_return",data,buf_len);
}
uint8_t senser_cmd[][10] = {
    {0xf1,0xf2,0xf3,0xf4,0xf5}
};
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
