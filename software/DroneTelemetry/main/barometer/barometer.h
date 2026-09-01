#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include <stdbool.h>

esp_err_t barometer_init(i2c_master_bus_handle_t bus, uint32_t freq);

esp_err_t barometer_read(float *pressure_pa, float *temperature_c);

#endif // BAROMETER_H
