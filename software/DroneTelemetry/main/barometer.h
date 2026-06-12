#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"


esp_err_t barometer_power_up_test(i2c_master_bus_handle_t i2c_bus, uint32_t i2c_freq_hz);



#endif // BAROMETER_H
