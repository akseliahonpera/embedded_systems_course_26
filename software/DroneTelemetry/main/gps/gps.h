#ifndef GPS_H
#define GPS_H

#include "esp_err.h"

/**
 * @brief Initialize the M20048 module
 *
 *
 */
esp_err_t gps_init();

void gps_read();

#endif /*GPS_H*/