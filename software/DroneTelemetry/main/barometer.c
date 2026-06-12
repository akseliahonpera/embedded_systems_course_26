#include "barometer.h"

static const char *TAG = "BAROMETER";

static i2c_master_dev_handle_t bmp581_dev;

#define BMP581_ADDR 0x46
#define BMP581_REG_CHIP_ID 0x01
#define BMP581_REG_STATUS 0x28
#define BMP581_REG_INT_STATUS 0x27
#define BMP581_STATUS_NVM_RDY (1 << 1)
#define BMP581_STATUS_NVM_ERR (1 << 2)
#define BMP581_INT_STATUS_POR 0x10
#define BMP581_CHIP_ID_VALUE 0x50
#define BMP581_I2C_TIMEOUT_MS 1000

static esp_err_t bmp581_read_register(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(
        bmp581_dev,
        &reg,
        sizeof(reg),
        value,
        sizeof(*value),
        BMP581_I2C_TIMEOUT_MS);
}

esp_err_t barometer_power_up_test(i2c_master_bus_handle_t i2c_bus, uint32_t i2c_freq_hz)
{
    i2c_device_config_t bmp581_i2c_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP581_ADDR,
        .scl_speed_hz = i2c_freq_hz,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &bmp581_i2c_cfg, &bmp581_dev), TAG, "Failed to add BMP581");

    uint8_t dummy = 0;
    ESP_RETURN_ON_ERROR(bmp581_read_register(BMP581_REG_CHIP_ID, &dummy), TAG, "Dummy read failed");

    uint8_t chip_id;
    ESP_RETURN_ON_ERROR(bmp581_read_register(BMP581_REG_CHIP_ID, &chip_id), TAG, "CHIP_ID read failed");

    ESP_LOGI(TAG, "CHIP_ID = 0x%02X", chip_id);
    if (chip_id != BMP581_CHIP_ID_VALUE)
    {
        ESP_LOGE(TAG, "Invalid CHIP_ID");
        return ESP_ERR_INVALID_RESPONSE;
    }


    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(bmp581_read_register(BMP581_REG_STATUS, &status), TAG, "STATUS read failed");


    ESP_LOGI(TAG, "STATUS = 0x%02X", status);
    if ((status & BMP581_STATUS_NVM_ERR) == 0x4)
    {
        ESP_LOGE(TAG, "NVM error");
        return ESP_FAIL;
    }
    if ((status & BMP581_STATUS_NVM_RDY) == 0x0)
    {
        ESP_LOGE(TAG, "NVM is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t int_status;
    ESP_RETURN_ON_ERROR(bmp581_read_register(BMP581_REG_INT_STATUS, &int_status), TAG, "INT_STATUS read failed");

    
    ESP_LOGI(TAG, "INT_STATUS = 0x%02X", int_status);
    if (int_status != BMP581_INT_STATUS_POR)
    {
        // This ok if and only if only esp32 was reseted and bmp581 was still powered i.e. you reprogrammed esp32
        // This is due to reading of int status register zeroing it.
        ESP_LOGW(TAG, "POR bit is not set in INT_STATUS");
    }

    return ESP_OK;
}