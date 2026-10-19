#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "types.h"

#include "barometer.h"
#include "imu.h"
#include "gps.h"

#include "telemetry.h"
#include "fusion.h"

#include "wifi_sta.h"

static const char *TAG = "MAIN";

// I2C confs
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000

#define WIFI_SSID       "PLACEHOLDER"
#define WIFI_PASSWORD   "PLACEHOLDER"
#define WIFI_TIMEOUT    100

void app_main(void)
{
    // Initialize wifi
    wifi_sta_start(WIFI_SSID, WIFI_PASSWORD, WIFI_TIMEOUT);

    // Init I2C
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

    // Initialize queues
    // Queue for sensor messages from sensor tasks to sensorfusion task
    QueueHandle_t fusion_queue = xQueueCreate(10, sizeof(sensor_msg_t));
    // Queue for fused data from sensorfusion task to telemetry task
    QueueHandle_t telemetry_queue = xQueueCreate(10, sizeof(fusion_msg_t));

    // Initialize fusion and telemetry tasks
    fusion_init(fusion_queue, telemetry_queue);
    telemetry_init(telemetry_queue);

    // Initialize sensor tasks
    gps_init(fusion_queue);
    vTaskDelay(pdMS_TO_TICKS(1000));
    barometer_init(fusion_queue, i2c_bus, I2C_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_init(fusion_queue, i2c_bus);
}
