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

static TaskHandle_t gps_task_handle;

// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;

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

        xQueueSend(fusion_queue, &msg, 0);
    }
    free(data);
}

esp_err_t gps_init(QueueHandle_t fusion_queue_handle)
{
    fusion_queue = fusion_queue_handle;
    // Init UART
    // I think a TX buffer is not needed since there is not that much traffic going from mcu to gps.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, 0, 0));

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                                 M20048_RX_GPIO, // ESP32 TX
                                 M20048_TX_GPIO, // ESP32 RX
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    BaseType_t task_created = xTaskCreate(gps_task, "GPS_TASK", 4096, NULL,
                                          5, &gps_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "GPS task started.");
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