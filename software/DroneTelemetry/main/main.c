#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "barometer.h"

static const char *TAG = "MAIN";

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000

void app_main(void)
{

    ESP_LOGI(TAG, "Älkää välittäkö pull-up varotuksesta, esp valittaa tästä aina jos sisäset pullupit disabloidaan, koska se ei voi tietää onko ulkosia pull-uppeja.");

    i2c_master_bus_handle_t i2c_bus;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_COUNT,
        .flags.enable_internal_pullup = false,
    };

    vTaskDelay(pdMS_TO_TICKS(3000)); // Allow sensor power-up

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* Initialize the barometer */
    ESP_ERROR_CHECK(barometer_init(i2c_bus, I2C_FREQ_HZ));

    while (1)
    {
        float pressure;
        float temperature;

        if (barometer_read(&pressure, &temperature) == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "Pressure: %.2f Pa (%.2f hPa), Temperature: %.2f C",
                     pressure,
                     pressure / 100.0f,
                     temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}