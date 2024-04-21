#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_SLAVE_ADDR 8
#define I2C_SLAVE_SDA_IO 21          /*!< GPIO number for I2C slave data  */
#define I2C_SLAVE_SCL_IO 22          /*!< GPIO number for I2C slave clock */
#define I2C_SLAVE_NUM I2C_NUM_0      /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN 1024    /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN 1024    /*!< I2C slave rx buffer size */

static uint8_t slave_tx_buffer[I2C_SLAVE_TX_BUF_LEN];
static uint8_t slave_rx_buffer[I2C_SLAVE_RX_BUF_LEN];

void i2c_slave_init() {
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = I2C_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_SLAVE_RX_BUF_LEN,
                       I2C_SLAVE_TX_BUF_LEN, 0);
}

void sendData() {
    char data[] = "Hello";
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, strlen(data), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_SLAVE_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void receiveData() {
    int data_length = i2c_slave_read_buffer(I2C_SLAVE_NUM, slave_rx_buffer, I2C_SLAVE_RX_BUF_LEN, portMAX_DELAY);
    if (data_length > 0) {
        printf("Data received from Master: %.*s\n", data_length, slave_rx_buffer);
    } else {
        printf("No signal from Master\n");
    }
}

void app_main() {
    i2c_slave_init();
    while(1) {
        sendData();
        receiveData();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
