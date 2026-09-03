#ifndef GPS_H
#define GPS_H

#include "esp_err.h"

/**
 * @brief Initialize the M20048 module and create the sensor task.
 *
 *
 */
esp_err_t gps_init(void);

void gps_read();

#endif /*GPS_H*/