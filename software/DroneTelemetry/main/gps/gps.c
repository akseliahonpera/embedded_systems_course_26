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
//#define UART_QUEUE_SIZE 10

// GPIOs
#define M20048_TX_GPIO GPIO_NUM_15 // esp rx
#define M20048_RX_GPIO GPIO_NUM_14 // esp tx
#define M20048_TM_GPIO GPIO_NUM_17 // not used as of now
#define M20048_FIX_GPIO GPIO_NUM_16
#define M20048_HW_R_GPIO GPIO_NUM_18 // not used as of now
#define M20048_HW_S_GPIO GPIO_NUM_13 // not used as of now

#define TIME_ZONE (+3)   //Oulu Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

static TaskHandle_t gps_task_handle;

// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;



// TODO
// Remove this if not needed
static void gps_task(void *arg)
{
    // TODO: NMEA parsetus (esim. nmea_parser, minmea tjsp), datan passaus fuusiotaskille
    // toistaseksi printtaa vaan terminaaliin 10 s välein
    (void)arg;

    sensor_msg_t msg;
    msg.type = SENSOR_GPS;

    uint8_t *data = malloc(512); // iso bufferi
    if (data == NULL)
    {
        ESP_LOGE(TAG, "vituixmän");
        vTaskDelete(NULL);
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));

        size_t buffered_len = 0;
        uart_get_buffered_data_len(UART_NUM, &buffered_len);


        if (buffered_len > 0)
        {
            size_t read_size = (buffered_len > (512 - 1)) ? (512 - 1) : buffered_len;

            int len = uart_read_bytes(UART_NUM, data, read_size, pdMS_TO_TICKS(50));

            if (len > 0)
            {
                data[len] = '\0';
                ESP_LOGI(TAG, "GPS data dump:\n%s", (char *)data);
            }
        }

        
        if (xQueueSend(fusion_queue, &msg, 0) != pdPASS) {
            ESP_LOGW(TAG, "Fusion queue full, no data added to queue");
        }
    }
    free(data);
}


/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;

    ESP_LOGI(TAG, "NMEA event received: %ld", (long)event_id);

    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
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
    if (nmea_handle == NULL) {
        ESP_LOGE(TAG, "NMEA parser initialization failed");
        return ESP_FAIL;
    }

    
    ESP_RETURN_ON_ERROR(nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL), TAG, "Could not register GPS event handler");


    // esp_err_t err = nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL);


    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Could not register GPS event handler: %s",
    //              esp_err_to_name(err));

    //     nmea_parser_deinit(nmea_handle);
    //     nmea_handle = NULL;
    //     return err;
    // }

    // ESP_LOGI(TAG, "NMEA parser initialized");

    // nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    // nmea_parser_handle_t nmea_handle = nmea_parser_init(&config);

    // vTaskDelay(pdMS_TO_TICKS(1000));

    // nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL);


    // Init UART
    // I think a TX buffer is not needed since there is not that much traffic going from mcu to gps.
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, 0, 0));

    // // Configure UART parameters
    // uart_config_t uart_config = {
    //     .baud_rate = UART_BAUD_RATE,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .rx_flow_ctrl_thresh = 122,
    // };
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));


    // // Set UART pins
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
    //                              M20048_RX_GPIO, // ESP32 TX
    //                              M20048_TX_GPIO, // ESP32 RX
    //                              UART_PIN_NO_CHANGE,
    //                              UART_PIN_NO_CHANGE,
    //                              UART_PIN_NO_CHANGE,
    //                              UART_PIN_NO_CHANGE));

    // BaseType_t task_created = xTaskCreate(gps_task, "GPS_TASK", 4096, NULL,
    //                                       5, &gps_task_handle);
    // if (task_created != pdPASS)
    // {
    //     return ESP_ERR_NO_MEM;
    // }
    // ESP_LOGI(TAG, "GPS task started.");
    return ESP_OK;
}

// left this here for debugging or stuff like that
void gps_read()
{
    uint8_t data[256];

    int len = uart_read_bytes(
        UART_NUM,
        data,
        sizeof(data) - 1,
        pdMS_TO_TICKS(1000));

    if (len > 0)
    {
        data[len] = '\0';
        ESP_LOGI("GPS", "Received: %s", (char *)data);
    }
}


