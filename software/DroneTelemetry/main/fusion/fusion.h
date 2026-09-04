#ifndef FUSION_H
#define FUSION_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief
 * 
 * @param fusion_queue_handle
 * @param telemetry_queue_handle
 * @return esp_err_t 
 */
esp_err_t fusion_init(QueueHandle_t fusion_queue_handle, QueueHandle_t telemetry_queue_handle);

#endif