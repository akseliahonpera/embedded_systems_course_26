#include "bno085_port.h"

#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sh2_err.h"

#define BNO085_ADDR 0x4A
#define BNO085_I2C_FREQ_HZ 100000
#define BNO085_TIMEOUT_MS 100
#define SHTP_HEADER_SIZE 4

static i2c_master_dev_handle_t bno085_dev;
static gpio_num_t bno085_hint_gpio;
static gpio_num_t bno085_nrst_gpio;
static gpio_num_t bno085_bootn_gpio;

static int hal_open(sh2_Hal_t *self);
static void hal_close(sh2_Hal_t *self);
static int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len,
                    uint32_t *timestamp_us);
static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len);
static uint32_t hal_get_time_us(sh2_Hal_t *self);

esp_err_t bno085_port_init(i2c_master_bus_handle_t bus,
                           gpio_num_t hint_gpio,
                           gpio_num_t nrst_gpio,
                           gpio_num_t bootn_gpio)
{
    if (bno085_dev != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bno085_hint_gpio = hint_gpio;
    bno085_nrst_gpio = nrst_gpio;
    bno085_bootn_gpio = bootn_gpio;

    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << nrst_gpio) | (1ULL << bootn_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), "BNO085", "GPIO setup failed");

    const gpio_config_t hint_config = {
        .pin_bit_mask = 1ULL << hint_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&hint_config), "BNO085", "HINT setup failed");

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO085_ADDR,
        .scl_speed_hz = BNO085_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, &bno085_dev);
}

bool bno085_port_data_ready(void)
{
    return bno085_dev != NULL && gpio_get_level(bno085_hint_gpio) == 0;
}

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    /* BOOTN high selects normal SH-2 firmware, rather than the bootloader. */
    gpio_set_level(bno085_nrst_gpio, 1);
    gpio_set_level(bno085_bootn_gpio, 1);
    esp_rom_delay_us(10000);
    gpio_set_level(bno085_nrst_gpio, 0);
    esp_rom_delay_us(10000);
    gpio_set_level(bno085_nrst_gpio, 1);
    esp_rom_delay_us(50000);
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    gpio_set_level(bno085_nrst_gpio, 0);
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len)
{
    (void)self;
    if (bno085_dev == NULL || len == 0 || len > SH2_HAL_MAX_TRANSFER_OUT) {
        return 0;
    }

    return i2c_master_transmit(bno085_dev, buffer, len, BNO085_TIMEOUT_MS) == ESP_OK
               ? (int)len
               : 0;
}

static int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len,
                    uint32_t *timestamp_us)
{
    if (!bno085_port_data_ready() || len < SHTP_HEADER_SIZE) {
        return 0;
    }

    esp_err_t err = i2c_master_receive(bno085_dev, buffer, SHTP_HEADER_SIZE,
                                       BNO085_TIMEOUT_MS);
    if (err != ESP_OK) {
        return 0;
    }

    const uint16_t packet_length =
        (((uint16_t)buffer[1] << 8) | buffer[0]) & 0x7FFF;
    if (packet_length < SHTP_HEADER_SIZE || packet_length > len ||
        packet_length > SH2_HAL_MAX_TRANSFER_IN) {
        return 0;
    }

    const uint16_t payload_length = packet_length - SHTP_HEADER_SIZE;
    if (payload_length > 0) {
        err = i2c_master_receive(bno085_dev, buffer + SHTP_HEADER_SIZE,
                                 payload_length, BNO085_TIMEOUT_MS);
        if (err != ESP_OK) {
            return 0;
        }
    }

    if (timestamp_us != NULL) {
        *timestamp_us = hal_get_time_us(self);
    }
    return packet_length;
}

sh2_Hal_t bno085_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_get_time_us,
};
