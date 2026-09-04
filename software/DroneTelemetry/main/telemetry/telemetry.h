#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief
 * 
 * @param telemetry_queue_handle 
 * @return esp_err_t 
 */
esp_err_t telemetry_init(QueueHandle_t telemetry_queue_handle);

#endif