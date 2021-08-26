
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "AHT20 log";

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define AHT20_ADDR                  0x38

#define AHT20_INIT                  0xBE
#define AHT20_INIT_PARAM_1          0x08
#define AHT20_INIT_PARAM_2          0x00

#define AHT20_TRIGGER               0xAC
#define AHT20_TRIGGER_PARAM_1       0x33
#define AHT20_TRIGGER_PARAM_2       0x00

#define AHT20_SOFT_RESET            0xBA

#define WRITE_BIT                   0
#define READ_BIT                    1

#define ACK_BOOL                    true
#define SEND_ACK                    I2C_MASTER_ACK
#define SEND_NACK_AT_END            I2C_MASTER_LAST_NACK

static esp_err_t i2c_master_init(void){
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

static esp_err_t aht20_intialize(){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDR << 1 | WRITE_BIT, ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_INIT , ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_INIT_PARAM_1 , ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_INIT_PARAM_2 , ACK_BOOL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(50 / portTICK_RATE_MS);
    return ret;
}

static esp_err_t aht20_trigger(){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDR << 1 | WRITE_BIT, ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_TRIGGER, ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_TRIGGER_PARAM_1 , ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_TRIGGER_PARAM_2 , ACK_BOOL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;

}

static esp_err_t aht20_get_status(uint8_t *status_byte){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDR << 1 | READ_BIT, ACK_BOOL);
    i2c_master_read_byte(cmd, status_byte, SEND_ACK);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t aht20_read_data_bytes(uint8_t *data){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_read(cmd, data, 4, SEND_NACK_AT_END);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

//UNUSED
static esp_err_t aht20_soft_reset(){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDR << 1 | WRITE_BIT, ACK_BOOL);
    i2c_master_write_byte(cmd, AHT20_SOFT_RESET, ACK_BOOL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;

}

static float temp_byte_to_float(uint8_t *data){

    uint32_t temp;
    temp = data[2] & 0x0f;
    temp <<= 8;
    temp += data[3];
    temp <<= 8;
    temp += data[4];

    return (float)temp/0x100000*200.0-50.0;
}

static float humidity_byte_to_float(uint8_t *data){

    uint32_t hum;
    hum = data[0];
    hum <<= 8;
    hum += data[1];
    hum <<= 4;
    hum += (data[2] >> 4);

    return (float)hum/0x100000*100;
}

void aht20_startup(){
    ESP_LOGI(TAG, "HELLO!");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_ERROR_CHECK(aht20_intialize());
    ESP_LOGI(TAG, "AHT20 INITIALIZED");
}


void get_data(float *temp, float *humidity){
    uint8_t status_byte;
    uint8_t data[6]; 
    ESP_ERROR_CHECK(aht20_trigger());
    vTaskDelay(80 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(aht20_get_status(&status_byte));
    if ((status_byte & 0x80) == 0x80){
        ESP_LOGI(TAG, "%x", status_byte);
        vTaskDelay(80 / portTICK_RATE_MS);
    }else{
        ESP_ERROR_CHECK(aht20_read_data_bytes(data));
        *temp = temp_byte_to_float(data);
        *humidity = humidity_byte_to_float(data);
    }
}


void app_main(void) {
    float temp;
    float humidity;
    aht20_startup();
    while(1){
        get_data(&temp, &humidity);
        ESP_LOGI(TAG, "Humidity : %.2f%%, Temperature: %.2f°C", humidity, temp);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
