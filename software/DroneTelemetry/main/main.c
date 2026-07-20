#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "barometer.h"
#include "imu.h"

static const char *TAG = "MAIN";

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000

void app_main(void)
{
    ESP_LOGI(TAG, "Älkää välittäkö pull-up varotuksesta...");

    i2c_master_bus_handle_t i2c_bus = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_COUNT,
        .flags = {
            .enable_internal_pullup = false,
        }};

    // Initialize the shared I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // 1. Give the entire hardware bus time to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialize the barometer first */
    ESP_LOGI(TAG, "Initializing Barometer...");
    ESP_ERROR_CHECK(barometer_init(i2c_bus, I2C_FREQ_HZ));
    ESP_LOGI(TAG, "Barometer initialized successfully.");

    // 2. CRUCIAL COOLDOWN DELAY
    // Let the barometer finish its boot up cycles and release its I2C hold
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Initialize the IMU */
    ESP_LOGI(TAG, "Initializing IMU...");
    ESP_ERROR_CHECK(imu_init(i2c_bus));
    ESP_LOGI(TAG, "IMU initialized successfully.");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // while (1)
    // {
    //     float pressure;
    //     float temperature;

    //     if (barometer_read(&pressure, &temperature) == ESP_OK)
    //     {
    //         ESP_LOGI(TAG,
    //                  "Pressure: %.2f Pa (%.2f hPa), Temperature: %.2f C",
    //                  pressure,
    //                  pressure / 100.0f,
    //                  temperature);
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(1000));

    // }
}