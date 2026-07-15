#ifndef BNO085_PORT_H
#define BNO085_PORT_H

#include "sh2_hal.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t bno085_port_init(i2c_master_bus_handle_t bus, uint32_t i2c_freq_hz);

// This is the HAL object to be used in imu.c
// See sh2_hal.h for details on the implementation
extern sh2_Hal_t bno085_hal;

#endif