#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#define BAUD_RATE 115200
#define I2C_SLAVE_ADDR 8
#define I2C_MASTER_SCL_IO 22          /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 21          /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0      /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main() {
    char data[32];
    i2c_master_init();
    uart_set_baudrate(UART_NUM_0, BAUD_RATE);

    while (1) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, "Hello", 20, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 20, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        printf("Data received from SLAVE: %s\n", data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
