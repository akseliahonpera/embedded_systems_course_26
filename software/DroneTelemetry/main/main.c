#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "barometer.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000


void app_main(void)
{

    vTaskDelay(pdMS_TO_TICKS(100)); // time delay to allow for devices to power-up

    i2c_master_bus_handle_t i2c_bus;    // i2c bus handle

    // i2c bus config
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_COUNT,
        .flags.enable_internal_pullup = false, // use external 2.2k pull-ups on pcb.
    };

    // init i2c bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // test barometer connection
    ESP_ERROR_CHECK(barometer_power_up_test(i2c_bus, I2C_FREQ_HZ));


}
