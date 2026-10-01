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
#include "esp_timer.h"

static const char *TAG = "GPS";

// UART confs
#define UART_BUF_SIZE (1024 * 2) // 2kb
#define UART_NUM (UART_NUM_2)    // UART 2 is unassigned, so use that for GPS
#define UART_BAUD_RATE 9600
// #define UART_QUEUE_SIZE 10

// GPIOs
#define M20048_TX_GPIO GPIO_NUM_15   // esp rx
#define M20048_RX_GPIO GPIO_NUM_14   // esp tx
#define M20048_TM_GPIO GPIO_NUM_17   // not used as of now, pulse per second
#define M20048_FIX_GPIO GPIO_NUM_16  // fix, not used
#define M20048_HW_R_GPIO GPIO_NUM_18 // reset
#define M20048_HW_S_GPIO GPIO_NUM_13 // standby

#define TIME_ZONE (+3)   // Oulu Time
#define YEAR_BASE (2000) // date in GPS starts from 2000

// Task handle
static TaskHandle_t gps_task_handle;

// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;

// Flag for printing data
static bool print_gps_data;

static void gps_task(void *arg)
{
    (void)arg;

    while (1)
    {
        uint32_t events;
        BaseType_t received = xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

        if (received == pdTRUE)
        {
            if (events & GPS_CMD_STANDBY)
            {
                /* Standby mode is a power saving mode that shuts down the RF section of the
                module and puts the processor into a standby mode. The RTC is kept alive, and
                the RAM power is maintained to keep the module configuration.
                The standby state can be initiated either with a hardware signal to Pin26 or by
                using a command.
                Hardware controlled Standby: Enable standby mode by a low state to Pin26
                (HW_S). To wake the module back to full power a high state needs to be
                applied to Pin26. If Pin26 is not to be used, then it must be kept floating (not
                connected).
                Standby mode command: Software on the host needs to send the “PMTK161”
                command through the UART interface*/
                esp_err_t err = gpio_set_level(M20048_HW_S_GPIO, 0);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Could not set GPS standby mode", esp_err_to_name(err));
                    // vTaskDelete(NULL); <- jatkossa vois tappaa taskin, miten uudelleenkäynnistys
                }
                ESP_LOGI(TAG, "GPS entered standby mode", NULL);
                // TODO: notify caller
            }

            if (events & GPS_CMD_EXIT_STANDBY)
            {
                /*To wake the module back to full power a high state needs to be
                applied to Pin26. If Pin26 is not to be used, then it must be kept floating (not
                connected).*/
                esp_err_t err = gpio_set_level(M20048_HW_S_GPIO, 1);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Could not exit GPS standby mode", esp_err_to_name(err));
                    // vTaskDelete(NULL); <- jatkossa vois tappaa taskin, miten uudelleenkäynnistys
                }
                ESP_LOGI(TAG, "GPS exited standby mode", NULL);
                // TODO: notify caller
                
            }

            if (events & GPS_CMD_RESET)
            {
                /*The External reset pin is default high by an internal 75Kohm and should be
                left floating if not used. To initiate a reset the pin needs to be pulled low. The
                module also initiates a reset if the VCC drops below the minimum 2.8V supply. */
                esp_err_t err = gpio_set_level(M20048_HW_R_GPIO, 0);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Could not pull GPS reset down", esp_err_to_name(err));
                    // vTaskDelete(NULL); <- jatkossa vois tappaa taskin, miten uudelleenkäynnistys
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                err = gpio_set_level(M20048_HW_R_GPIO, 1);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Could not release GPS reset", esp_err_to_name(err));
                    // vTaskDelete(NULL); <- jatkossa vois tappaa taskin, miten uudelleenkäynnistys
                }
                ESP_LOGI(TAG, "GPS reset completed", NULL);
                // TODO: notify caller
            }

            if (events & GPS_CMD_PRINT_STATUS)
            {
                print_gps_data ^= 1;
                ESP_LOGI(TAG, "GPS print status: %d", print_gps_data);
            }
        }
    }
}

void gps_notify(gps_command_t cmd)
{
    xTaskNotify(
        gps_task_handle,
        cmd,
        eSetBits);
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

    gps_t *gps = NULL;

    switch (event_id)
    {

    case GPS_UNKNOWN:
        ESP_LOGW(TAG, "Disabled or unknown NMEA statement: %s", event_data != NULL ? (char *)event_data : "(null)");
        break;

    case GPS_UPDATE:
        gps = (gps_t *)event_data;

        const bool has_fix =
            gps->valid &&
            gps->fix != GPS_FIX_INVALID &&
            gps->fix_mode != GPS_MODE_INVALID;

        if (print_gps_data)
        {
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
                .tim = gps->tim,
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

    // Config reset and standby pins //

    const gpio_config_t standby_config = {
        .pin_bit_mask = 1ULL << M20048_HW_S_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&standby_config), TAG, "GPS standby GPIO setup failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(M20048_HW_S_GPIO, 1), TAG, "Could not set GPS to normal mode");

    const gpio_config_t hw_reset_config = {
        .pin_bit_mask = 1ULL << M20048_HW_R_GPIO,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&hw_reset_config), TAG, "GPS hw_reset GPIO setup failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(M20048_HW_R_GPIO, 1), TAG, "Could not set GPS hw_reset to 1");

    // Create the GPS task //

    BaseType_t task_created = xTaskCreate(gps_task, "GPS_TASK", 4096, NULL,
                                          5, &gps_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GPS task started.");
    return ESP_OK;
}
