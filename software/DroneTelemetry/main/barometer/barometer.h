#ifndef BAROMETER_H
#define BAROMETER_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t barometer_init(QueueHandle_t fusion_queue_handle, i2c_master_bus_handle_t bus, uint32_t freq);

/*
for debugging
*/
esp_err_t barometer_read(float *pressure_pa, float *temperature_c);

#endif // BAROMETER_H
