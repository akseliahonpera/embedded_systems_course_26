#include "barometer.h"
#include "bmp5.h"
#include "bmp5_selftest.h"
#include "bmp581_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "types.h"

static const char *TAG = "BAROMETER";

// API device from bmp5_defs.h. Callbacks go into member variables.
static struct bmp5_dev bmp_dev;
static struct bmp5_osr_odr_press_config osr_odr_press_cfg;

static TaskHandle_t barometer_task_handle;
static QueueHandle_t fusion_queue;

static void barometer_task(void *arg)
{
    // print every 10th second
    (void)arg;
    struct bmp5_sensor_data sensor_data;
    float pressure_pa, temperature_c;
    sensor_msg_t msg;
    msg.type = SENSOR_BARO;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int8_t rslt = bmp5_get_sensor_data(
            &sensor_data,
            &osr_odr_press_cfg,
            &bmp_dev);

        if (rslt != BMP5_OK)
        {
            ESP_LOGE(TAG, "BMP581 read failed (%d)", rslt);
        }
        pressure_pa = sensor_data.pressure;
        temperature_c = sensor_data.temperature;

        ESP_LOGI(TAG,
                 "Pressure: %.2f Pa (%.2f hPa), Temperature: %.2f C",
                 pressure_pa,
                 pressure_pa / 100.0f,
                 temperature_c);

        xQueueSend(fusion_queue, &msg, 0);
    }
}

esp_err_t barometer_init(QueueHandle_t fusion_queue_handle, i2c_master_bus_handle_t bus, uint32_t freq)
{
    fusion_queue = fusion_queue_handle;

    ESP_RETURN_ON_ERROR(bmp581_port_init(bus, freq),
                        TAG,
                        "Port init failed");

    ESP_RETURN_ON_ERROR(bmp581_soft_reset(),
                        TAG,
                        "BMP581 reset failed");

    // Tässä laitetaan callbackit HALista (bmp581_port.h) API-laitteeseen
    bmp_dev.intf = BMP5_I2C_INTF;
    bmp_dev.read = bmp581_i2c_read;
    bmp_dev.write = bmp581_i2c_write;
    bmp_dev.delay_us = bmp581_delay_us;
    bmp_dev.intf_ptr = NULL;

    int8_t rslt = bmp5_init(&bmp_dev);

    if (rslt != BMP5_OK)
    {
        ESP_LOGE(TAG, "bmp5_init failed (%d)", rslt);
        return ESP_FAIL;
    }

    rslt = bmp5_selftest_check(&bmp_dev);

    if (rslt != BMP5_OK)
    {
        ESP_LOGE(TAG, "BMP581 self-test failed (%d)", rslt);
        return ESP_FAIL;
    }

    osr_odr_press_cfg = (struct bmp5_osr_odr_press_config){
        .osr_t = BMP5_OVERSAMPLING_1X,
        .osr_p = BMP5_OVERSAMPLING_4X,
        .odr = BMP5_ODR_50_HZ,
        .press_en = BMP5_ENABLE,
    };

    rslt = bmp5_set_osr_odr_press_config(
        &osr_odr_press_cfg,
        &bmp_dev);

    if (rslt != BMP5_OK)
    {
        ESP_LOGE(TAG, "Sensor configuration failed (%d)", rslt);
        return ESP_FAIL;
    }

    rslt = bmp5_set_power_mode(
        BMP5_POWERMODE_NORMAL,
        &bmp_dev);

    if (rslt != BMP5_OK)
    {
        ESP_LOGE(TAG, "Power mode failed (%d)", rslt);
        return ESP_FAIL;
    }

    BaseType_t task_created = xTaskCreate(barometer_task, "BAROMETER_TASK", 4096, NULL,
                                          5, &barometer_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Barometer task started.");
    return ESP_OK;
}

// left for debugging
esp_err_t barometer_read(float *pressure_pa, float *temperature_c)
{
    struct bmp5_sensor_data sensor_data;

    int8_t rslt = bmp5_get_sensor_data(
        &sensor_data,
        &osr_odr_press_cfg,
        &bmp_dev);

    if (rslt != BMP5_OK)
    {
        ESP_LOGE(TAG, "BMP581 read failed (%d)", rslt);
        return ESP_FAIL;
    }

    if (pressure_pa != NULL)
    {
        *pressure_pa = sensor_data.pressure;
    }

    if (temperature_c != NULL)
    {
        *temperature_c = sensor_data.temperature;
    }

    return ESP_OK;
}