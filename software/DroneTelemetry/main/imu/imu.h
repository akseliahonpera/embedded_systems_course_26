#ifndef IMU_H
#define IMU_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Initialize the BNO085 and start reporting rotation-vector samples.
 *
 * The BNO085 must be connected using the GPIO mapping in imu.c.
 */
esp_err_t imu_init(QueueHandle_t sensor_queue_handle, i2c_master_bus_handle_t i2c_handle);

#endif /* IMU_H */
