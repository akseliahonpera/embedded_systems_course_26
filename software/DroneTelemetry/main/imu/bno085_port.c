#include "sh2.h"

#include <string.h>
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "freertos/task.h"
#include "sh2_hal.h"
#include "sh2.h"
#include "sh2_err.h"
#include "esp_timer.h"

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

// Static callbacks (private to this file)
static int hal_open(sh2_Hal_t *self);
static void hal_close(sh2_Hal_t *self);
static int hal_read(sh2_Hal_t *self,
                    uint8_t *buffer,
                    unsigned len,
                    uint32_t *timestamp_us);
static int hal_write(sh2_Hal_t *self,
                     uint8_t *buffer,
                     unsigned len);
static uint32_t hal_getTimeUs(sh2_Hal_t *self);

static int hal_open(sh2_Hal_t *self)
{
    (void)self;
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static int hal_write(sh2_Hal_t *self,
                     uint8_t *buffer,
                     unsigned len)
{
    (void)self;

    // This works a bit different from the bmp581 write. The SH2 library gives
    // the whole SHTP packet, which is ready to send as is.
    esp_err_t err =
        i2c_master_transmit(
            bno085_dev,
            buffer,
            len,
            BNO085_TIMEOUT_MS);

    if (err != ESP_OK)
        return 0;

    return len;
}

static int hal_read(sh2_Hal_t *self,
                    uint8_t *buffer,
                    unsigned len,
                    uint32_t *timestamp_us)
{
    esp_err_t err = i2c_master_receive(
        bno085_dev,
        buffer,
        SH2_HAL_MAX_TRANSFER_IN,
        BNO085_TIMEOUT_MS);

    if (err != ESP_OK)
        return 0;

    uint16_t packetLength =
        ((uint16_t)buffer[1] << 8 | buffer[0]) & 0x7FFF;

    if (packetLength > len)
        return 0;

    *timestamp_us = esp_timer_get_time();

    return packetLength;
}

// Global HAL object
sh2_Hal_t bno085_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs,
};
