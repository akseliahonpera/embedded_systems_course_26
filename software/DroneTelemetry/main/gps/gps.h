#ifndef GPS_H
#define GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/**
 * @brief Initialize the M20048 module, NMEA parser, and terminal menu task.
 *
 *
 */
esp_err_t gps_init(QueueHandle_t fusion_queue_handle);

/**
 * @brief Send one GPS UART command after gps_init() succeeds.
 *
 * Pass a complete checksummed sentence, e.g. "$PMTK605*31". CRLF is added
 * automatically (an existing line ending is accepted). Maximum wire length
 * is 255 bytes including CRLF. Call from task context, not an ISR.
 *
 * Writes are serialized with a mutex. Waits up to one second for the mutex
 * and up to one second for UART TX completion. ESP_OK means transmitted,
 * not acknowledged by the module. Replies use the existing NMEA event handler.
 * No commands are sent automatically; the terminal menu calls this function.
 * Returns ESP_ERR_INVALID_STATE before initialization, ESP_ERR_INVALID_ARG
 * for malformed packets, ESP_ERR_INVALID_CRC for bad checksums,
 * ESP_ERR_INVALID_SIZE for oversized packets, ESP_FAIL for an incomplete write,
 * or ESP_ERR_TIMEOUT if the mutex or UART transmission times out.
 */
esp_err_t gps_send_command(const char *packet);

/**
 * For debugging
 */
void gps_read();

#endif /*GPS_H*/
