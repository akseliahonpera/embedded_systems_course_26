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
typedef enum {
    IMU_DATA_READY                          = (1UL << 0),
    IMU_CMD_START_BIAS_MEASUREMENT          = (1UL << 1),
    IMU_CMD_START_COVARIANCE_MEASUREMENT    = (1UL << 2),
    IMU_CMD_MODE_STANDBY                    = (1UL << 3),
    IMU_CMD_MODE_NORMAL                     = (1UL << 4)
} imu_command_t;

typedef enum {
    IMU_MEASURING_BIAS,
    IMU_NORMAL,
    IMU_MEASURING_COVARIANCE
} imu_state_t;

esp_err_t imu_init(i2c_master_bus_handle_t i2c_handle);

void imu_notify(imu_command_t cmd);


#endif /* IMU_H */
