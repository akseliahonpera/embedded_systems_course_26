#ifndef BAROMETER_H
#define BAROMETER_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Initializes the barometer
 * 
 * @param bus The I2C bus to assign the bmp5 to
 * @param freq I2C frequency
 * @return esp_err_t 
 */
esp_err_t barometer_init(i2c_master_bus_handle_t bus, uint32_t freq);

/**
 * @brief Sends a command to the barometer
 * 
 * @param cmd The command to send
 */
void barometer_notify(barometer_command_t cmd);

/**
 * @brief Returns the barometer state
 * 
 * @return barometer_state_t 
 */
barometer_state_t barometer_get_state(void);

typedef enum {
    BAROMETER_CMD_EXIT_STANDBY = (1UL << 0),
    BAROMETER_CMD_STANDBY      = (1UL << 1),
    BAROMETER_CMD_RESET        = (1UL << 2)
} barometer_command_t;

typedef enum {
    BAROMETER_STATE_STANDBY      = (1UL << 0),
    BAROMETER_STATE_RUNNING      = (1UL << 1),
    BAROMETER_STATE_INIT         = (1UL << 2)
} barometer_state_t;

#endif // BAROMETER_H