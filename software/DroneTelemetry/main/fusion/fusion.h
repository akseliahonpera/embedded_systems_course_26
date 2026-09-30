#ifndef FUSION_H
#define FUSION_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    FUSION_CMD_START        = (1UL << 0),
    FUSION_CMD_STOP         = (1UL << 1),
    FUSION_CMD_PRINT_STATUS = (1UL << 2),
    FUSION_CMD_TOGGLE_PERIODIC_STATUS = (1UL << 3),
    FUSION_CMD_TOGGLE_IMU_COVARIANCE_SOURCE = (1UL << 4)
} fusion_command_t;

// Task-context commands are combined with eSetBits, just like imu_notify().
// START resets the estimate and waits for a fresh GPS fix. STOP wins if both
// bits arrive together. PRINT_STATUS runs after the state-changing command.
// TOGGLE_PERIODIC_STATUS enables/disables periodic output in every filter mode.
// TOGGLE_IMU_COVARIANCE_SOURCE switches between manual and latest received values.
// Queue mode is the default and falls back to manual values until data arrives.
void fusion_notify(fusion_command_t cmd);

/**
 * @brief
 * 
 * @param fusion_queue_handle
 * @param telemetry_queue_handle
 * @return esp_err_t 
 */
esp_err_t fusion_init(QueueHandle_t fusion_queue_handle, QueueHandle_t telemetry_queue_handle);

#endif
