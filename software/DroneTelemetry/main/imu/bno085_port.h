#ifndef BNO085_PORT_H
#define BNO085_PORT_H

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "sh2_hal.h"

/**
 * @brief Initialize the BNO085 I2C device and its control/status GPIOs.
 *
 * BOOTN is driven high to boot the normal SH-2 firmware. HINT is active low.
 */
esp_err_t bno085_port_init(i2c_master_bus_handle_t bus,
                           gpio_num_t hint_gpio,
                           gpio_num_t nrst_gpio,
                           gpio_num_t bootn_gpio);

/** @brief True when the BNO085 has an SHTP packet ready to be read. */
bool bno085_port_data_ready(void);

extern sh2_Hal_t bno085_hal;

#endif /* BNO085_PORT_H */
