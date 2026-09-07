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
#define GPS_MAX_CHANNELS 40 // Maximum five-digit fields in one PMTK packet
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

typedef struct {
    uint8_t satellite;
    uint8_t snr;
    uint8_t status;
} gps_channel_t;

typedef struct {
    size_t count;
    gps_channel_t channels[GPS_MAX_CHANNELS];
} gps_channel_report_t;

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

typedef enum {
    GPS_CHN_UNKNOWN,
    GPS_CHN_DISABLED,
    GPS_CHN_ENABLED,
    GPS_CHN_ENABLING,
    GPS_CHN_DISABLING,
} gps_channel_state_t;

static atomic_int gps_channel_state = GPS_CHN_UNKNOWN;

// Configuration readback is distinct from GGA fix quality (GPS_FIX_DGPS).
static atomic_int gps_dgps_mode = -1;
static atomic_int gps_sbas_enabled = -1;
static atomic_int gps_dgps_ack = -1;
static atomic_int gps_sbas_ack = -1;
static atomic_bool gps_dgps_requested = false;

static void gps_invalidate_dgps(void)
{
    atomic_store(&gps_dgps_mode, -1);
    atomic_store(&gps_sbas_enabled, -1);
    atomic_store(&gps_dgps_ack, -1);
    atomic_store(&gps_sbas_ack, -1);
}

static const char *gps_dgps_mode_name(void)
{
    switch (atomic_load(&gps_dgps_mode)) {
    case 0: return "NONE (confirmed)";
    case 1: return "RTCM (confirmed)";
    case 2: return "SBAS (confirmed)";
    default: return "UNKNOWN (readback not received)";
    }
}

static const char *gps_sbas_name(void)
{
    switch (atomic_load(&gps_sbas_enabled)) {
    case 0: return "OFF (confirmed)";
    case 1: return "ON (confirmed)";
    default: return "UNKNOWN (readback not received)";
    }
}

static const char *gps_channel_state_name(void)
{
    switch (atomic_load(&gps_channel_state)) {
    case GPS_CHN_DISABLED: return "DISABLED (confirmed)";
    case GPS_CHN_ENABLED: return "ENABLED (confirmed)";
    case GPS_CHN_ENABLING: return "UNKNOWN (waiting for enable acknowledgment)";
    case GPS_CHN_DISABLING: return "UNKNOWN (waiting for disable acknowledgment)";
    default: return "UNKNOWN (not confirmed since startup/reset)";
    }
}

// PMTK314 field 18 controls PMTKCHN; zero disables it, nonzero enables it.
static int gps_channel_request_state(const char *packet)
{
    const char *field = packet + 9; // Skip "$PMTK314,".
    for (int i = 0; i < 18; i++) {
        field += strcspn(field, ",*");
        if (*field != ',') {
            return GPS_CHN_UNKNOWN; // Includes the restore-defaults command.
        }
        field++;
    }
    size_t digits = strspn(field, "0123456789");
    if (digits == 0 || (field[digits] != '*' && field[digits] != ',')) {
        return GPS_CHN_UNKNOWN;
    }
    return strspn(field, "0") == digits ? GPS_CHN_DISABLING : GPS_CHN_ENABLING;
}

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

static esp_err_t gps_parse_channel_report(const char *reply, gps_channel_report_t *report)
{
    gps_uart_packet_t packet;
    report->count = 0;
    esp_err_t err = gps_prepare_packet(reply, &packet);
    if (err != ESP_OK) {
        return err;
    }
    if (strncmp(packet.data, "$PMTKCHN,", 9) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *field = packet.data + 9;
    while (1) {
        size_t length = strcspn(field, ",*");
        if (length != 5 || report->count == GPS_MAX_CHANNELS) {
            return ESP_ERR_INVALID_SIZE;
        }
        for (size_t i = 0; i < length; i++) {
            if (field[i] < '0' || field[i] > '9') {
                return ESP_ERR_INVALID_ARG;
            }
        }
        // PMTKCHN uses SSNNF: satellite number, SNR in dB, channel status.
        gps_channel_t *channel = &report->channels[report->count++];
        channel->satellite = (field[0] - '0') * 10 + field[1] - '0';
        channel->snr = (field[2] - '0') * 10 + field[3] - '0';
        channel->status = field[4] - '0';
        field += length;
        if (*field == '*') {
            return ESP_OK;
        }
        field++; // Skip the comma before the next field.
    }
}

static void gps_print_channel_report(const char *reply)
{
    gps_channel_report_t report;
    esp_err_t err = gps_parse_channel_report(reply, &report);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Invalid PMTKCHN report: %s", esp_err_to_name(err));
        return;
    }

    // Only the NMEA parser task calls this function. Keep the table off its stack.
    static char rows[GPS_MAX_CHANNELS * 40 + 1];
    rows[0] = '\0';
    size_t used = 0;
    unsigned idle = 0, searching = 0, tracking = 0, unknown = 0, hidden = 0;
    unsigned strongest = 0, strongest_satellite = 0;
    for (size_t i = 0; i < report.count; i++) {
        const gps_channel_t *channel = &report.channels[i];
        const char *status;
        switch (channel->status) {
        case 0: status = "Idle"; idle++; break;
        case 1: status = "Searching"; searching++; break;
        case 2: status = "Tracking"; tracking++; break;
        default: status = "Unknown"; unknown++; break;
        }
        if (channel->snr > strongest) {
            strongest = channel->snr;
            strongest_satellite = channel->satellite;
        }
        if (channel->satellite == 0 && channel->snr == 0 && channel->status == 0) {
            hidden++;
            continue;
        }
        int written = snprintf(rows + used, sizeof(rows) - used,
                               " %2u |  %02u | %7u | %-9s (%u)\n",
                               (unsigned)i + 1, (unsigned)channel->satellite,
                               (unsigned)channel->snr, status, (unsigned)channel->status);
        if (written < 0 || (size_t)written >= sizeof(rows) - used) {
            ESP_LOGW(TAG, "GPS channel report formatting overflow");
            return;
        }
        used += (size_t)written;
    }
    char signal_summary[64];
    if (strongest > 0) {
        snprintf(signal_summary, sizeof(signal_summary), "Strongest reported SNR: %u dB (SV %02u)",
                 strongest, strongest_satellite);
    } else {
        snprintf(signal_summary, sizeof(signal_summary), "No channel reports a nonzero SNR");
    }
    ESP_LOGI(TAG,
             "\n================ GPS CHANNEL STATUS ================\n"
             " Channels: %u | Tracking: %u | Searching: %u | Idle: %u | Unknown: %u\n"
             " %s\n"
             " Ch |  SV | SNR(dB) | Status\n"
             "%s"
             " Empty idle channels omitted: %u\n"
             "====================================================",
             (unsigned)report.count, tracking, searching, idle, unknown,
             signal_summary, rows, hidden);
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
    bool sets_channel_output = strncmp(data, "$PMTK314,", 9) == 0;
    if (sets_channel_output) {
        int state = atomic_load(&gps_channel_state);
        if (state == GPS_CHN_ENABLING || state == GPS_CHN_DISABLING) {
            xSemaphoreGive(gps_tx_mutex);
            ESP_LOGW(TAG, "Still waiting for the previous NMEA output acknowledgment");
            return ESP_ERR_INVALID_STATE;
        }
        atomic_store(&gps_channel_state, gps_channel_request_state(data));
    }
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
        atomic_store(&gps_channel_state, GPS_CHN_UNKNOWN);
        gps_invalidate_dgps();
    }
    if (strncmp(data, "$PMTK301,", 9) == 0) {
        atomic_store(&gps_dgps_mode, -1);
        atomic_store(&gps_dgps_ack, -1);
    } else if (strncmp(data, "$PMTK313,", 9) == 0) {
        atomic_store(&gps_sbas_enabled, -1);
        atomic_store(&gps_sbas_ack, -1);
    }
    int written = uart_write_bytes(UART_NUM, data, length);
    esp_err_t err = written == (int)length
                        ? uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(1000))
                        : ESP_FAIL;
    if (sets_aic && err != ESP_OK) {
        atomic_store(&gps_aic_state, GPS_AIC_UNKNOWN);
    }
    if (sets_channel_output && err != ESP_OK) {
        atomic_store(&gps_channel_state, GPS_CHN_UNKNOWN);
    }
    if (err == ESP_OK && strncmp(data, "$PMTK104*", 9) == 0) {
        atomic_store(&gps_dgps_requested, true);
    } else if (err == ESP_OK && strncmp(data, "$PMTK161,", 9) == 0) {
        atomic_store(&gps_dgps_requested, false);
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

static void gps_update_settings_from_reply(const char *reply)
{
    gps_uart_packet_t packet;
    // Raw RX is still printed, but corrupt replies must not confirm a state.
    if (gps_prepare_packet(reply, &packet) != ESP_OK) {
        return;
    }
    if (strncmp(packet.data, "$PMTK010,001*", 13) == 0 ||
        strncmp(packet.data, "$PMTK011,", 9) == 0) {
        atomic_store(&gps_aic_state, GPS_AIC_UNKNOWN);
        atomic_store(&gps_channel_state, GPS_CHN_UNKNOWN);
        gps_invalidate_dgps();
        atomic_store(&gps_dgps_requested, true);
        return;
    }
    if (packet.length == sizeof("$PMTK501,2*28\r\n") - 1 &&
        strncmp(packet.data, "$PMTK501,", 9) == 0 &&
        packet.data[9] >= '0' && packet.data[9] <= '2') {
        atomic_store(&gps_dgps_mode, packet.data[9] - '0');
        ESP_LOGI(TAG, "DGPS source: %s", gps_dgps_mode_name());
        return;
    }
    if (packet.length == sizeof("$PMTK513,1*28\r\n") - 1 &&
        strncmp(packet.data, "$PMTK513,", 9) == 0 &&
        (packet.data[9] == '0' || packet.data[9] == '1')) {
        atomic_store(&gps_sbas_enabled, packet.data[9] - '0');
        ESP_LOGI(TAG, "SBAS reception: %s", gps_sbas_name());
        return;
    }
    if (packet.length == sizeof("$PMTK001,301,3*32\r\n") - 1 &&
        (strncmp(packet.data, "$PMTK001,301,", 13) == 0 ||
         strncmp(packet.data, "$PMTK001,313,", 13) == 0) &&
        packet.data[13] >= '0' && packet.data[13] <= '3') {
        atomic_int *ack = packet.data[10] == '0' ? &gps_dgps_ack : &gps_sbas_ack;
        int status = packet.data[13] - '0';
        atomic_store(ack, status);
        if (status != 3) {
            ESP_LOGW(TAG, "DGPS/SBAS command rejected (ACK %d: 0 invalid, 1 unsupported, 2 failed)", status);
        }
        return;
    }
    if (strncmp(packet.data, "$PMTKCHN,", 9) == 0) {
        int state = atomic_load(&gps_channel_state);
        // Reports buffered before an output change must not consume its ACK.
        if (state != GPS_CHN_ENABLING && state != GPS_CHN_DISABLING) {
            atomic_compare_exchange_strong(&gps_channel_state, &state, GPS_CHN_ENABLED);
        }
        return;
    }
    if (packet.length == sizeof("$PMTK001,314,3*36\r\n") - 1 &&
        strncmp(packet.data, "$PMTK001,314,", 13) == 0 &&
        packet.data[13] >= '0' && packet.data[13] <= '3') {
        int pending = atomic_load(&gps_channel_state);
        if (pending == GPS_CHN_ENABLING || pending == GPS_CHN_DISABLING) {
            int confirmed = packet.data[13] == '3'
                                ? (pending == GPS_CHN_ENABLING ? GPS_CHN_ENABLED : GPS_CHN_DISABLED)
                                : GPS_CHN_UNKNOWN;
            if (atomic_compare_exchange_strong(&gps_channel_state, &pending, confirmed)) {
                ESP_LOGI(TAG, "PMTKCHN channel reports: %s (ACK status %c)",
                         gps_channel_state_name(), packet.data[13]);
            }
        }
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

// Runs in the console task; the parser continues receiving replies and fixes.
static void gps_configure_dgps(void)
{
    static const struct {
        const char *packet;
        atomic_int *reply;
    } commands[] = {
        {"$PMTK313,1*2E", &gps_sbas_ack},
        {"$PMTK301,2*2E", &gps_dgps_ack},
        {"$PMTK413*34", &gps_sbas_enabled},
        {"$PMTK401*37", &gps_dgps_mode},
    };
    // Allow startup/wake notifications to settle before sending settings.
    vTaskDelay(pdMS_TO_TICKS(1000));
    atomic_store(&gps_dgps_requested, false);
    gps_invalidate_dgps();
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        atomic_store(commands[i].reply, -1);
        esp_err_t err = gps_send_command(commands[i].packet);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "DGPS configuration TX failed: %s", esp_err_to_name(err));
            break;
        }
        int64_t deadline = esp_timer_get_time() + 2000000;
        while (atomic_load(commands[i].reply) == -1 && esp_timer_get_time() < deadline) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (atomic_load(commands[i].reply) == -1) {
            ESP_LOGW(TAG, "No reply to %s", commands[i].packet);
        }
    }
    if (atomic_load(&gps_dgps_mode) == 2 && atomic_load(&gps_sbas_enabled) == 1) {
        ESP_LOGI(TAG, "SBAS DGPS configuration confirmed; Fix: DGPS indicates corrections are actually in use.");
    } else {
        ESP_LOGW(TAG, "SBAS DGPS not confirmed: source=%s, SBAS=%s. Press d to retry.",
                 gps_dgps_mode_name(), gps_sbas_name());
    }
}

static void gps_print_menu(void)
{
    printf("\nGPS command menu\n"
           "  1  Standby\n"
           "  2  Wake from command standby\n"
           "  3  Query firmware version\n"
           "  4  Toggle PMTKCHN channel reports and printing (keeps standard NMEA enabled)\n"
           "  5  Enable interference cancellation\n"
           "  6  Full cold restart (clears GPS configuration)\n"
           "  7  Toggle GPS_UPDATE printing\n"
           "  8  Send PMTK test packet\n"
           "  9  Disable interference cancellation\n"
           "  d  Configure and verify SBAS DGPS\n"
           "  m  Show this menu\n"
           "GPS_UPDATE printing: %s\n"
           "Active Interference Cancellation: %s\n"
           "PMTKCHN channel reports: %s\n"
           "DGPS source: %s | SBAS reception: %s\n"
           "Actual corrections in use: see Fix: DGPS in GPS_UPDATE\n"
           "Press a key to select an action.\n",
           atomic_load_explicit(&gps_update_print_enabled, memory_order_relaxed) ? "ON" : "OFF",
           gps_aic_state_name(), gps_channel_state_name(), gps_dgps_mode_name(), gps_sbas_name());
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
            atomic_store(&gps_dgps_requested, true);
            ESP_LOGI(TAG, "GPS wake bytes sent; allow the module to resume before the next command.");
        } else {
            ESP_LOGE(TAG, "GPS wake failed: %s", esp_err_to_name(err));
        }
        return;
    }
    case '3':
        packet = "$PMTK605*31";
        break;
    case '4': {
        int state = atomic_load(&gps_channel_state);
        if (state == GPS_CHN_ENABLING || state == GPS_CHN_DISABLING) {
            ESP_LOGW(TAG, "Wait for the channel report acknowledgment before toggling again");
            return;
        }
        // Keep GLL, RMC, VTG, GGA, GSA and GSV enabled in both modes.
        // If the current state is unknown, establish DISABLED first.
        if (state == GPS_CHN_UNKNOWN) {
            ESP_LOGI(TAG, "PMTKCHN state unknown; requesting DISABLED first");
        }
        packet = state == GPS_CHN_DISABLED
                     ? "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1*29"
                     : "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28";
        break;
    }
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
    case 'd':
    case 'D':
        atomic_store(&gps_dgps_requested, true);
        return;
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
        if (atomic_exchange(&gps_dgps_requested, false)) {
            gps_configure_dgps();
        }
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
        if (reply != NULL && strncmp(reply, "$PMTKCHN,", 9) == 0) {
            // Print reports whenever the GPS sends them; option 4 controls transmission.
            gps_print_channel_report(reply);
            gps_update_settings_from_reply(reply);
            break;
        }
        if (reply != NULL && strncmp(reply, "$PMTK", 5) == 0) {
            // Command replies remain visible when GPS_UPDATE printing is off.
            ESP_LOGI(TAG, "UART RX: %.*s", (int)strcspn(reply, "\r\n"), reply);
            gps_update_settings_from_reply(reply);
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
    atomic_store(&gps_dgps_requested, !set_to_standby);
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
