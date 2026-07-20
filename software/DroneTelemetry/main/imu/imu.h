#ifndef IMU_H
#define IMU_H


#include "esp_err.h"

/**
 * @brief Initializes the BNO085 IMU hardware, opens the SH2 connection,
 *        and spawns the background processing task on Core 1.
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t imu_init(i2c_master_bus_handle_t i2c_handle);

#endif // IMU_H