#include <esp_event_base.h>

///Wifi network credentials HERE ->>
#define EXAMPLE_ESP_WIFI_SSID      "wifi_ssid"
#define EXAMPLE_ESP_WIFI_PASS      "password"
#define EXAMPLE_ESP_MAXIMUM_RETRY  20
///Following 3 according to the wifi network specs, these should work with modern wifi routers
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK



static void event_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data);

void wifi_init_sta();

void start_wifi();
