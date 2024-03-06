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
#define I2C_MASTER_SCL0_IO           (GPIO_NUM_5)      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA0_IO           (GPIO_NUM_6)      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM0              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_SCL1_IO           (GPIO_NUM_7)      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA1_IO           (GPIO_NUM_8)      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */


#define AS600_SENSOR_ADDR                 0x36        
#define AS5600_ANGLE_H           0x0e      
#define AS5600_ANGLE_L           0x0f
#define AS5600_MAX_VALUE         4096


static as5600_data as5600_data0,as5600_data1;
static int as5600_circle=0;
uint16_t as5600_value,as5600_last_value0,as5600_last_value1,as5600_init_value0,as5600_init_value1;
int as5600_dir,as5600_diff;
float as5600_angle,as5600_total_angle;

// uint16_t as5600_get_value(i2c_port_t i2c_master_port){
//     uint16_t result = 0;
//     uint8_t data[2];
//     as5600_dev_read(I2C_MASTER_NUM0,AS5600_ANGLE_H, &data[0], 1);
//     as5600_dev_read(I2C_MASTER_NUM0,AS5600_ANGLE_L, &data[1], 1);
//     // ESP_LOG_BUFFER_HEX(TAG,data,2);
//     result=(uint16_t)(data[0]<<8|data[1]); //一共就11位 注意
//     return result;
// }
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
int as5600_get_circle(int dir,int circle){
    int temp_circle = circle;
    switch (dir)
    {
    case AS5600_TURN_N_OVER:
        temp_circle += 1;
        /* code */
        break;
    case AS5600_TURN_P_OVER:
        temp_circle -= 1;
        break;
    default:
        break;
    }
    return temp_circle;
}
float as5600_get_total_angle(uint16_t value,int circle){
    float angle = as5600_get_angle(value);
    return (float)circle*360+angle;
}


void as5600_app_dev_measure(as5600_data *data){
    data->angle = as5600_get_angle(data->value);
    data->direction = as5600_get_dir(data->value - data->last_value);
    data->circle = as5600_get_circle(data->direction,data->circle);
    data->totol_angle = as5600_get_total_angle(data->value-data->init_value,data->circle);
    data->last_value = data->value;
    // ESP_LOGI(TAG, "angle:%f dir:%d circle:%d total:%.2f",data->angle,data->direction,data->circle,data->totol_angle);
}
void as5600_app_monitor(as5600_data data0, as5600_data data1){
    ESP_LOGI(TAG, "angle:%f dir:%d circle:%d total:%.2f  |  angle:%f dir:%d circle:%d total:%.2f" 
        ,data0.angle,data0.direction,data0.circle,data0.totol_angle
        ,data1.angle,data1.direction,data1.circle,data1.totol_angle);
}
void as5600_app_task(void *arg){
    uint8_t data[2];
    as5600_data0.init_value = as5600_dev_iic0_read();
    as5600_data1.init_value = as5600_dev_iic1_read();
    while (1){
        as5600_data0.value = as5600_dev_iic0_read();
        as5600_data1.value = as5600_dev_iic1_read();
        as5600_app_dev_measure(&as5600_data0);
        as5600_app_dev_measure(&as5600_data1);
        as5600_app_monitor(as5600_data0,as5600_data1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void as5600_app_task_creat(void){
    xTaskCreate(as5600_app_task, "as5600_app_task", 1024*3, NULL, configMAX_PRIORITIES-1, &as5600_app_task_Handle);
}

void as5600_app_init(){
    as5600_dev_init();
    
    ESP_LOGI(TAG, "AS5600 I2C initialized successfully");
    as5600_app_task_creat(); 
}