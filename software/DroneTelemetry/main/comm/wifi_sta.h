#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up WiFi in station mode and block until an IP is obtained.
 *
 * Initialises NVS, the netif and the event loop, so call it once before
 * anything else. Auto-reconnects in the background after the initial
 * connection succeeds.
 *
 * @param ssid       network name
 * @param password   WPA2 passphrase, NULL or "" for an open network
 * @param timeout_ms how long to wait for an IP before giving up
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no IP was obtained
 */
esp_err_t wifi_sta_start(const char *ssid, const char *password, uint32_t timeout_ms);

/** @brief True while the station holds an IP address. */
bool wifi_sta_is_connected(void);

#ifdef __cplusplus
}
#endif
