#include "sh2.h"

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "freertos/task.h"
#include "sh2_hal.h"

#define BNO085_ADDR 0x4A
#define BNO085_TIMEOUT_MS 1000

static i2c_master_dev_handle_t bno085_dev = NULL;

esp_err_t bno085_port_init(i2c_master_bus_handle_t bus, uint32_t i2c_freq_hz)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO085_ADDR,
        .scl_speed_hz = i2c_freq_hz,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, &bno085_dev);
}

