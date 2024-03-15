/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define AS600_SENSOR_ADDR                 0x36        
#define AS5600_ANGLE_H           0x0e      
#define AS5600_ANGLE_L           0x0f
#define AS5600_MAX_VALUE         4096

enum AS5600_DIR_ENUM{
    AS5600_STOP = 0,
    AS5600_TURN_N = 1,
    AS5600_TURN_P = 2,
    AS5600_TURN_N_OVER = 3,
    AS5600_TURN_P_OVER = 4,
};

static int as5600_circle=0;


/************************算法层************************************/
// 定义线性模型的系数和截距
double coefficient = 5.85353152e-06;
double intercept = -0.0742029745713122;

// 定义函数来预测值
double predict(double x) {
    return coefficient * x + intercept;
}
/*****************************************************************/



/************************算法层************************************/
// 定义线性模型的系数和截距
double coefficient = 5.85353152e-06;
double intercept = -0.0742029745713122;

// 定义函数来预测值
double predict(double x) {
    return coefficient * x + intercept;
}
/*****************************************************************/


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t as5600_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, AS600_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void as5600_read_angle(uint8_t reg_addr){
    int ret;
    uint8_t angle_high,angle_low,angle_h_reg,angle_l_reg;
    float angle;
    angle_h_reg = 0x0e;
    angle_l_reg = 0x0f;
    angle_high = i2c_master_write_to_device(I2C_MASTER_NUM, reg_addr, 0x0e, sizeof(angle_high), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    // angle_low = i2c_master_write_to_device(I2C_MASTER_NUM, reg_addr, angle_l_reg, sizeof(uint8_t), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    // angle = (angle_high<<8)|angle_low;
    ESP_LOGI(TAG,"%d",angle_high);
}
uint16_t as5600_get_value(uint8_t *data){
    uint16_t result = 0;
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
int as5600_get_total_value(uint16_t value,int circle){
    return (int)circle*4096+value;
}
void app_main(void)
{
    uint8_t data[2];
    uint16_t as5600_value,as5600_value_last;
    int as5600_dir,as5600_diff,as5600_total_value;
    float as5600_angle,as5600_total_angle;
    int64_t as5600_time;
    double as5600_weight;
    double as5600_weight;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "AS5600 I2C initialized successfully");

    as5600_register_read(AS5600_ANGLE_H, &data[0], 1);
    as5600_register_read(AS5600_ANGLE_L, &data[1], 1);
    as5600_value_last = as5600_get_value(&data);

    while (1)
    {
        // ReadAngle();
        as5600_time = esp_timer_get_time();
        as5600_register_read(AS5600_ANGLE_H, &data[0], 1);
        as5600_register_read(AS5600_ANGLE_L, &data[1], 1);
        as5600_value = as5600_get_value(&data);
        as5600_angle = as5600_get_angle(as5600_value);
        as5600_diff = as5600_value - as5600_value_last;
        as5600_dir = as5600_get_dir(as5600_diff);
        as5600_get_circle(as5600_dir);
        as5600_total_value = as5600_get_total_value(as5600_value,as5600_circle);
        as5600_value_last = as5600_value;
        as5600_weight = predict(as5600_total_value);
        printf("%lld,%d,%d,%f\n",as5600_time,as5600_value,as5600_total_value,as5600_weight);
        // ESP_LOGI(TAG, "diff:%d angle:%f dir:%d circle:%d total:%.2f",as5600_diff,as5600_angle,as5600_dir,as5600_circle,as5600_total_angle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
