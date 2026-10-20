#ifndef GPS_H
#define GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

typedef enum {
    GPS_CMD_EXIT_STANDBY        = (1UL << 0),
    GPS_CMD_STANDBY         = (1UL << 1),
    GPS_CMD_RESET        = (1UL << 2),
    GPS_CMD_PRINT_STATUS = (1UL << 3)
} gps_command_t;

typedef enum {
    GPS_STATE_STANDBY   = (1UL << 0),
    GPS_STATE_RUNNING   = (1UL << 1),
    GPS_STATE_INIT      = (1UL << 2)
} gps_state_t;

/**
 * @brief Initialize the M20048 module and create the sensor task.
 *
 * 
 */
esp_err_t gps_init();

/**
 * @brief Pass a command to the GPS task
 * 
 * @param cmd 
 */
void gps_notify(gps_command_t cmd);

/**
 * @brief Return gps state
 * 
 * @return gps_state_t 
 */
gps_state_t gps_get_state(void);

#endif /*GPS_H*/