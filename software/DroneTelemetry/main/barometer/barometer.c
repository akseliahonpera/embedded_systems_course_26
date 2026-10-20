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
#include "fusion.h"

static const char *TAG = "BAROMETER";

// API device from bmp5_defs.h. Callbacks go into member variables.
static struct bmp5_dev bmp_dev;
static struct bmp5_osr_odr_press_config osr_odr_press_cfg;

// Barometer state
static barometer_state_t barometer_state;

// Task handle
static TaskHandle_t barometer_task_handle;

void barometer_notify(barometer_command_t cmd)
{
    xTaskNotify(
        barometer_task_handle,
        cmd,
        eSetBits);
}

barometer_state_t barometer_get_state(void) {
    return barometer_state;
}

static void barometer_task(void *arg)
{
    (void)arg;
    struct bmp5_sensor_data sensor_data;
    sensor_msg_t msg;
    msg.type = SENSOR_BARO;

    while (1)
    {
        uint32_t events;
        BaseType_t received = xTaskNotifyWait(0, UINT32_MAX, &events, pdMS_TO_TICKS(5000));

        if (received == pdTRUE) {
            if (events & BAROMETER_CMD_STANDBY) {
                if (bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp_dev) == BMP5_OK) {
                    barometer_state = BAROMETER_STATE_STANDBY;
                    ESP_LOGI(TAG, "Barometer entered standby mode");
                } else {
                    ESP_LOGI(TAG, "Barometer failed to enter standby mode");
                }
            }
            if (events & BAROMETER_CMD_EXIT_STANDBY) {
                if (bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp_dev) == BMP5_OK) {
                    barometer_state = BAROMETER_STATE_RUNNING;
                    ESP_LOGI(TAG, "Barometer exited standby mode");
                } else {
                    ESP_LOGI(TAG, "Barometer failed to exit standby mode");
                }
            }
            if (events & BAROMETER_CMD_RESET) {
                barometer_state = BAROMETER_STATE_INIT;
                int8_t rslt = bmp5_soft_reset(&bmp_dev);
                if (rslt != BMP5_OK) {
                    ESP_LOGE(TAG, "BMP581 reset failed (%d)", rslt);
                } else {
                    rslt = bmp5_set_osr_odr_press_config(
                        &osr_odr_press_cfg,
                        &bmp_dev);

                    if (rslt != BMP5_OK) {
                        ESP_LOGE(TAG, "Sensor configuration after reset failed (%d)", rslt);
                    } else {
                        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp_dev);
                        if (rslt != BMP5_OK) {
                            ESP_LOGE(TAG, "Power mode after reset failed (%d)", rslt);
                        } else {
                            barometer_state = BAROMETER_STATE_RUNNING;
                            ESP_LOGI(TAG, "Barometer reset successful");
                        }
                    }
                }
            }
        }

        if (barometer_state == BAROMETER_STATE_RUNNING) {
            int8_t rslt = bmp5_get_sensor_data(
                &sensor_data,
                &osr_odr_press_cfg,
                &bmp_dev);

            if (rslt != BMP5_OK)
            {
                ESP_LOGE(TAG, "BMP581 read failed (%d)", rslt);
            } else {
                msg.data.baro.pressure = sensor_data.pressure;
                msg.data.baro.temperature = sensor_data.temperature;

                fusion_send_data(&msg);
            }
        }
    }
}

esp_err_t barometer_init(i2c_master_bus_handle_t bus, uint32_t freq)
{
    barometer_state = BAROMETER_STATE_INIT;
    ESP_RETURN_ON_ERROR(bmp581_port_init(bus, freq),
                        TAG,
                        "Port init failed");

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
    barometer_state = BAROMETER_STATE_RUNNING;
    ESP_LOGI(TAG, "Barometer task started.");
    return ESP_OK;
}