#include "gps.h"
#include "driver/uart.h"
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "types.h"
#include "nmea_parser.h"

static const char *TAG = "GPS";

// UART confs
#define UART_BUF_SIZE (1024 * 2) // 2kb
#define UART_NUM (UART_NUM_2)    // UART 2 is unassigned, so use that for GPS
#define UART_BAUD_RATE 9600
// #define UART_QUEUE_SIZE 10

// GPIOs
#define M20048_TX_GPIO GPIO_NUM_15 // esp rx
#define M20048_RX_GPIO GPIO_NUM_14 // esp tx
#define M20048_TM_GPIO GPIO_NUM_17 // not used as of now
#define M20048_FIX_GPIO GPIO_NUM_16
#define M20048_HW_R_GPIO GPIO_NUM_18 // not used as of now
#define M20048_HW_S_GPIO GPIO_NUM_13 // not used as of now

#define TIME_ZONE (+3)   // Oulu Time
#define YEAR_BASE (2000) // date in GPS starts from 2000

static TaskHandle_t gps_task_handle;

// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;

static const char *gps_fix_name(gps_fix_t fix)
{
    switch (fix)
    {
    case GPS_FIX_GPS:
        return "GPS";
    case GPS_FIX_DGPS:
        return "DGPS";
    case GPS_FIX_INVALID:
    default:
        return "invalid";
    }
}

static const char *gps_mode_name(gps_fix_mode_t mode)
{
    switch (mode)
    {
    case GPS_MODE_2D:
        return "2D";
    case GPS_MODE_3D:
        return "3D";
    case GPS_MODE_INVALID:
    default:
        return "invalid";
    }
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)event_handler_arg;
    (void)event_base;

    gps_t *gps = NULL;

    switch (event_id)
    {

    case GPS_UNKNOWN:
        ESP_LOGW(TAG, "Disabled or unknown NMEA statement: %s", event_data != NULL ? (char *)event_data : "(null)");
        break;

    case GPS_UPDATE:
        gps = (gps_t *)event_data;

        ESP_LOGI(TAG, "%s", "--------------------------------------------------------------------------------");

        ESP_LOGI(TAG,
                 "Validity: RMC=%s, fix=%s, mode=%s",
                 gps->valid ? "valid" : "invalid",
                 gps_fix_name(gps->fix),
                 gps_mode_name(gps->fix_mode));

        ESP_LOGI(TAG,
                 "Time: %02u:%02u:%02u.%03u UTC",
                 (unsigned)gps->tim.hour,
                 (unsigned)gps->tim.minute,
                 (unsigned)gps->tim.second,
                 (unsigned)gps->tim.thousand);

        ESP_LOGI(TAG,
                 "Position: latitude=%.7f°, longitude=%.7f°",
                 gps->latitude,
                 gps->longitude);

        ESP_LOGI(TAG,
                 "Altitude: %.2f m",
                 gps->altitude);

        ESP_LOGI(TAG,
                 "Motion: speed=%.3f m/s, course=%.2f°, variation=%.2f°",
                 gps->speed,
                 gps->cog,
                 gps->variation);

        ESP_LOGI(TAG,
                 "DOP: HDOP=%.2f, PDOP=%.2f, VDOP=%.2f",
                 gps->dop_h,
                 gps->dop_p,
                 gps->dop_v);

        ESP_LOGI(TAG,
                 "Satellites: in-use=%u, in-view=%u",
                 (unsigned)gps->sats_in_use,
                 (unsigned)gps->sats_in_view);

        ESP_LOGI(TAG, "%s", "--------------------------------------------------------------------------------");
        break;
    default:
        break;
    }
}

esp_err_t gps_init(QueueHandle_t fusion_queue_handle)
{
    fusion_queue = fusion_queue_handle;
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();

    config.uart.uart_port = UART_NUM;
    config.uart.rx_pin = M20048_TX_GPIO;
    config.uart.baud_rate = UART_BAUD_RATE;
    config.uart.data_bits = UART_DATA_8_BITS;
    config.uart.parity = UART_PARITY_DISABLE;
    config.uart.stop_bits = UART_STOP_BITS_1;

    nmea_parser_handle_t nmea_handle = NULL;
    nmea_handle = nmea_parser_init(&config);
    if (nmea_handle == NULL)
    {
        ESP_LOGE(TAG, "NMEA parser initialization failed");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL), TAG, "Could not register GPS event handler");

    ESP_LOGI(TAG, "GPS task started.");
    return ESP_OK;
}
