#include "bmp581_port.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "bmp5.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP581_ADDR 0x46
#define BMP581_I2C_TIMEOUT_MS 1000

static i2c_master_dev_handle_t bmp581_dev = NULL;

esp_err_t bmp581_port_init(i2c_master_bus_handle_t bus, uint32_t i2c_freq_hz)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP581_ADDR,
        .scl_speed_hz = i2c_freq_hz,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, &bmp581_dev);
}

BMP5_INTF_RET_TYPE bmp581_i2c_read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    (void)intf_ptr;

    esp_err_t err = i2c_master_transmit_receive(
        bmp581_dev,
        &reg_addr,
        1,
        reg_data,
        len,
        BMP581_I2C_TIMEOUT_MS);

    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}

BMP5_INTF_RET_TYPE bmp581_i2c_write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    (void)intf_ptr;

    uint8_t tx[len + 1]; // sized for data plus register address

    tx[0] = reg_addr;
    memcpy(&tx[1], reg_data, len); // copy data into buffer

    esp_err_t err = i2c_master_transmit(
        bmp581_dev,
        tx,
        len + 1,
        BMP581_I2C_TIMEOUT_MS);

    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}

void bmp581_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

esp_err_t bmp581_soft_reset(void)
{
    uint8_t reset_cmd[2] = {
        0x7E, // BMP581 CMD register
        0xB6  // Soft reset command
    };

    esp_err_t err = i2c_master_transmit(
        bmp581_dev,
        reset_cmd,
        sizeof(reset_cmd),
        BMP581_I2C_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        return err;
    }

    // BMP581 needs time after reset
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}