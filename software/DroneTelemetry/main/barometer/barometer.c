#include "barometer.h"
#include "bmp5.h"
#include "bmp5_selftest.h"
#include "bmp581_port.h"

static const char *TAG = "BAROMETER";

// API device from bmp5_defs.h. Callbacks go into member variables.
static struct bmp5_dev bmp_dev;
static struct bmp5_osr_odr_press_config osr_odr_press_cfg;

esp_err_t barometer_init(i2c_master_bus_handle_t bus, uint32_t freq)
{
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

    return ESP_OK;
}

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