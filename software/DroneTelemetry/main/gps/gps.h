#ifndef GPS_H
#define GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/**
 * @brief Initialize the M20048 module and create the sensor task.
 *
 *
 */
esp_err_t gps_init(QueueHandle_t fusion_queue_handle);

/**
 * For debugging
 */
void gps_read();

#endif /*GPS_H*/