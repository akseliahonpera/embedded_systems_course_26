#include "gps.h"
#include "driver/uart.h"
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "types.h"
#include "nmea_parser.h"
#include "esp_timer.h"

static const char *TAG = "GPS";

// UART confs
#define UART_BUF_SIZE (1024 * 2) // 2kb
#define UART_NUM (UART_NUM_2)    // UART 2 is unassigned, so use that for GPS
#define UART_BAUD_RATE 9600
#define GPS_UART_PACKET_SIZE 256 // Maximum 255-byte PMTK packet plus NUL
// #define UART_QUEUE_SIZE 10

// GPIOs
#define M20048_TX_GPIO GPIO_NUM_15 // esp rx
#define M20048_RX_GPIO GPIO_NUM_14 // esp tx
#define M20048_TM_GPIO GPIO_NUM_17 // not used as of now
#define M20048_FIX_GPIO GPIO_NUM_16
#define M20048_HW_R_GPIO GPIO_NUM_18 // active-low hardware reset
#define M20048_HW_S_GPIO GPIO_NUM_13 // active-low hardware standby

#define TIME_ZONE (+3)   // Oulu Time
#define YEAR_BASE (2000) // date in GPS starts from 2000


// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;

typedef struct {
    size_t length;
    char data[GPS_UART_PACKET_SIZE];
} gps_uart_packet_t;

static SemaphoreHandle_t gps_tx_mutex;
static atomic_bool gps_update_print_enabled = true;

typedef enum {
    GPS_AIC_UNKNOWN,
    GPS_AIC_OFF,
    GPS_AIC_ON,
    GPS_AIC_ENABLING,
    GPS_AIC_DISABLING,
} gps_aic_state_t;

static atomic_int gps_aic_state = GPS_AIC_UNKNOWN;

static const char *gps_aic_state_name(void)
{
    switch (atomic_load(&gps_aic_state)) {
    case GPS_AIC_OFF: return "OFF (confirmed)";
    case GPS_AIC_ON: return "ON (confirmed)";
    case GPS_AIC_ENABLING: return "UNKNOWN (waiting for enable acknowledgment)";
    case GPS_AIC_DISABLING: return "UNKNOWN (waiting for disable acknowledgment)";
    default: return "UNKNOWN (not confirmed since startup/reset)";
    }
}

// Copy one checksummed sentence and normalize its line ending to CRLF.
static esp_err_t gps_prepare_packet(const char *packet, gps_uart_packet_t *message)
{
    if (packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t length = strnlen(packet, GPS_UART_PACKET_SIZE);
    if (length == GPS_UART_PACKET_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    while (length > 0 && (packet[length - 1] == '\r' || packet[length - 1] == '\n')) {
        length--;
    }
    if (length + 3 > sizeof(message->data)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (length < 5 || packet[0] != '$' || packet[length - 3] != '*') {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t checksum = 0;
    for (size_t i = 1; i < length - 3; i++) {
        unsigned char c = (unsigned char)packet[i];
        if (c < 0x20 || c > 0x7e || c == '$' || c == '*') {
            return ESP_ERR_INVALID_ARG;
        }
        checksum ^= c;
    }
    static const char hex[] = "0123456789ABCDEF";
    if (toupper((unsigned char)packet[length - 2]) != hex[checksum >> 4] ||
        toupper((unsigned char)packet[length - 1]) != hex[checksum & 0x0f]) {
        return ESP_ERR_INVALID_CRC;
    }

    memcpy(message->data, packet, length);
    memcpy(message->data + length, "\r\n", 3);
    message->length = length + 2;
    return ESP_OK;
}

static esp_err_t gps_write_uart(const char *data, size_t length)
{
    if (gps_tx_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(gps_tx_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    bool sets_aic = length == sizeof("$PMTK286,1*23\r\n") - 1 &&
                    strncmp(data, "$PMTK286,", 9) == 0 &&
                    (data[9] == '0' || data[9] == '1');
    if (sets_aic) {
        int state = atomic_load(&gps_aic_state);
        if (state == GPS_AIC_ENABLING || state == GPS_AIC_DISABLING) {
            xSemaphoreGive(gps_tx_mutex);
            ESP_LOGW(TAG, "Still waiting for the previous AIC acknowledgment");
            return ESP_ERR_INVALID_STATE;
        }
        // Record the requested mode before TX so a fast ACK can confirm it.
        atomic_store(&gps_aic_state, data[9] == '1' ? GPS_AIC_ENABLING : GPS_AIC_DISABLING);
    } else if (strncmp(data, "$PMTK104*", 9) == 0 || strncmp(data, "$PMTK161,", 9) == 0) {
        atomic_store(&gps_aic_state, GPS_AIC_UNKNOWN);
    }
    int written = uart_write_bytes(UART_NUM, data, length);
    esp_err_t err = written == (int)length
                        ? uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(1000))
                        : ESP_FAIL;
    if (sets_aic && err != ESP_OK) {
        atomic_store(&gps_aic_state, GPS_AIC_UNKNOWN);
    }
    xSemaphoreGive(gps_tx_mutex);
    return err;
}

esp_err_t gps_send_command(const char *packet)
{
    gps_uart_packet_t message = {0};
    esp_err_t err = gps_prepare_packet(packet, &message);
    if (err != ESP_OK) {
        return err;
    }
    err = gps_write_uart(message.data, message.length);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "UART TX: %.*s", (int)message.length - 2, message.data);
    }
    return err;
}

static void gps_update_aic_from_reply(const char *reply)
{
    gps_uart_packet_t packet;
    // Raw RX is still printed, but corrupt replies must not confirm a state.
    if (gps_prepare_packet(reply, &packet) != ESP_OK) {
        return;
    }
    if (strncmp(packet.data, "$PMTK010,001*", 13) == 0 ||
        strncmp(packet.data, "$PMTK011,", 9) == 0) {
        atomic_store(&gps_aic_state, GPS_AIC_UNKNOWN);
        return;
    }
    if (packet.length != sizeof("$PMTK001,286,3*3C\r\n") - 1 ||
        strncmp(packet.data, "$PMTK001,286,", 13) != 0 ||
        packet.data[13] < '0' || packet.data[13] > '3') {
        return;
    }
    int pending = atomic_load(&gps_aic_state);
    if (pending != GPS_AIC_ENABLING && pending != GPS_AIC_DISABLING) {
        return; // An ACK does not contain the requested on/off mode.
    }
    int confirmed = packet.data[13] == '3'
                        ? (pending == GPS_AIC_ENABLING ? GPS_AIC_ON : GPS_AIC_OFF)
                        : GPS_AIC_UNKNOWN;
    if (atomic_compare_exchange_strong(&gps_aic_state, &pending, confirmed)) {
        ESP_LOGI(TAG, "Active Interference Cancellation: %s (ACK status %c)",
                 gps_aic_state_name(), packet.data[13]);
    }
}

static void gps_print_menu(void)
{
    printf("\nGPS command menu\n"
           "  1  Standby\n"
           "  2  Wake from command standby\n"
           "  3  Query firmware version\n"
           "  4  Set selected NMEA output (disables GLL/VTG)\n"
           "  5  Enable interference cancellation\n"
           "  6  Full cold restart (clears GPS configuration)\n"
           "  7  Toggle GPS_UPDATE printing\n"
           "  8  Send PMTK test packet\n"
           "  9  Disable interference cancellation\n"
           "  m  Show this menu\n"
           "GPS_UPDATE printing: %s\n"
           "Active Interference Cancellation: %s\n"
           "Press a key to select an action.\n",
           atomic_load_explicit(&gps_update_print_enabled, memory_order_relaxed) ? "ON" : "OFF",
           gps_aic_state_name());
    fflush(stdout);
}

static void gps_handle_menu_key(int key)
{
    const char *packet = NULL;
    switch (key) {
    case '1':
        packet = "$PMTK161,0*28";
        break;
    case '2': {
        // Any UART byte wakes the M20048 from software-command standby.
        esp_err_t err = gps_write_uart("\r\n", 2);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "GPS wake bytes sent; allow the module to resume before the next command.");
        } else {
            ESP_LOGE(TAG, "GPS wake failed: %s", esp_err_to_name(err));
        }
        return;
    }
    case '3':
        packet = "$PMTK605*31";
        break;
    case '4':
        packet = "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0*35";
        break;
    case '5':
        packet = "$PMTK286,1*23";
        break;
    case '6':
        packet = "$PMTK104*37";
        break;
    case '7': {
        bool enabled = !atomic_load_explicit(&gps_update_print_enabled, memory_order_relaxed);
        atomic_store_explicit(&gps_update_print_enabled, enabled, memory_order_relaxed);
        ESP_LOGI(TAG, "GPS_UPDATE printing %s", enabled ? "enabled" : "disabled");
        return;
    }
    case '8':
        packet = "$PMTK000*32";
        break;
    case '9':
        packet = "$PMTK286,0*22";
        break;
    case 'm':
    case 'M':
        gps_print_menu();
        return;
    case '\r':
    case '\n':
    case ' ':
    case '\t':
        return;
    default:
        ESP_LOGW(TAG, "Unknown GPS menu key. Press m for the menu.");
        return;
    }
    esp_err_t err = gps_send_command(packet);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPS command failed: %s", esp_err_to_name(err));
    }
}

static void gps_console_task(void *arg)
{
    (void)arg;
    gps_print_menu();
    while (1) {
        // stdin is the ESP32 console, not the GPS UART owned by the parser.
        int key = getchar();
        if (key == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        gps_handle_menu_key(key);
    }
}

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

    int static counter = 0;

    gps_t *gps = NULL;

    switch (event_id)
    {

    case GPS_UNKNOWN: {
        const char *reply = (const char *)event_data;
        if (reply != NULL && strncmp(reply, "$PMTK", 5) == 0) {
            // Command replies remain visible when GPS_UPDATE printing is off.
            ESP_LOGI(TAG, "UART RX: %.*s", (int)strcspn(reply, "\r\n"), reply);
            gps_update_aic_from_reply(reply);
        } else {
            ESP_LOGW(TAG, "Disabled or unknown NMEA statement: %s", reply != NULL ? reply : "(null)");
        }
        break;
    }

    case GPS_UPDATE:
        gps = (gps_t *)event_data;

        const bool has_fix =
            gps->valid &&
            gps->fix != GPS_FIX_INVALID &&
            gps->fix_mode != GPS_MODE_INVALID;

        if (atomic_load_explicit(&gps_update_print_enabled, memory_order_relaxed)) {
            ESP_LOGI(TAG,
                     "\n"
                     "===================== GPS UPDATE =====================\n"
                     " Navigation : %-7s | Fix: %-7s | Mode: %s\n"
                     " UTC date   : %04u-%02u-%02u\n"
                     " UTC time   : %02u:%02u:%02u.%03u\n"
                     " Position   : lat=% .7f deg, lon=% .7f deg\n"
                     " Altitude   : %.2f m\n"
                     " Motion     : %.3f m/s, course=%.2f deg, variation=%.2f deg\n"
                     " Precision  : HDOP=%.2f, PDOP=%.2f, VDOP=%.2f\n"
                     " Satellites : %u used, %u in view\n"
                     "======================================================",
                     has_fix ? "VALID" : "INVALID",
                     gps_fix_name(gps->fix),
                     gps_mode_name(gps->fix_mode),
                     (unsigned)(YEAR_BASE + gps->date.year),
                     (unsigned)gps->date.month,
                     (unsigned)gps->date.day,
                     (unsigned)gps->tim.hour,
                     (unsigned)gps->tim.minute,
                     (unsigned)gps->tim.second,
                     (unsigned)gps->tim.thousand,
                     gps->latitude,
                     gps->longitude,
                     gps->altitude,
                     gps->speed,
                     gps->cog,
                     gps->variation,
                     gps->dop_h,
                     gps->dop_p,
                     gps->dop_v,
                     (unsigned)gps->sats_in_use,
                     (unsigned)gps->sats_in_view);
        }

        sensor_msg_t msg = {
            .type = SENSOR_GPS,
            .timestamp = esp_timer_get_time(),
            .data.gps = {
                .latitude = gps->latitude,
                .longtitude = gps->longitude,
                .has_fix = has_fix,
            },
        };

        if (xQueueSend(fusion_queue, &msg, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Fusion queue full; GPS update discarded");
        }

        break;
    default:
        break;
    }
}

esp_err_t gps_init(QueueHandle_t fusion_queue_handle)
{
    if (gps_tx_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << M20048_HW_R_GPIO,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };





    ESP_RETURN_ON_ERROR(gpio_config(&reset_config), TAG, "GPS reset GPIO setup failed");

    vTaskDelay(pdMS_TO_TICKS(1000));

    bool do_hardware_reset = true;
    bool set_to_standby = false;


    if (do_hardware_reset) {
        // Assert HW_R, then release it to the module's internal pull-up.
        ESP_RETURN_ON_ERROR(gpio_set_level(M20048_HW_R_GPIO, 0), TAG, "Could not assert GPS reset");
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_RETURN_ON_ERROR(gpio_set_level(M20048_HW_R_GPIO, 1), TAG, "Could not release GPS reset");
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "GPS reset done.");

    }




    if (set_to_standby)
    {
        const gpio_config_t standby_config = {
            .pin_bit_mask = 1ULL << M20048_HW_S_GPIO,
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&standby_config), TAG, "GPS standby GPIO setup failed");

        // Hold HW_S low to keep the module in hardware standby mode.
        ESP_RETURN_ON_ERROR(gpio_set_level(M20048_HW_S_GPIO, 0), TAG, "Could not set GPS standby mode");
        ESP_LOGI(TAG, "GPS hardware standby enabled.");
    }

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

    SemaphoreHandle_t tx_mutex = xSemaphoreCreateMutex();
    if (tx_mutex == NULL) {
        nmea_parser_deinit(nmea_handle);
        return ESP_ERR_NO_MEM;
    }

    // The parser owns RX; enable ESP32 TX on the GPS module's RX pin.
    esp_err_t err = uart_set_pin(UART_NUM, M20048_RX_GPIO, UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure GPS UART TX pin");
        goto uart_init_failed;
    }
    err = nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not register GPS event handler");
        goto uart_init_failed;
    }
    gps_tx_mutex = tx_mutex;
    if (xTaskCreate(gps_console_task, "gps_console", 3072, NULL, 5, NULL) != pdPASS) {
        gps_tx_mutex = NULL;
        ESP_LOGE(TAG, "Could not create GPS console task");
        err = ESP_ERR_NO_MEM;
        goto uart_init_failed;
    }

    ESP_LOGI(TAG, "GPS task started.");
    return ESP_OK;

uart_init_failed:
    nmea_parser_deinit(nmea_handle);
    vSemaphoreDelete(tx_mutex);
    return err;
}
