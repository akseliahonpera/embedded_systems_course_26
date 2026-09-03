#include "gps.h"
#include "driver/uart.h"
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "GPS";

// UART confs
#define UART_BUF_SIZE (1024 * 2) // 2kb
#define UART_NUM (UART_NUM_2)    // UART 2 is unassigned, so use that for GPS
#define UART_BAUD_RATE 9600
#define UART_QUEUE_SIZE 10

// GPIOs
#define M20048_TX_GPIO GPIO_NUM_15 // esp rx
#define M20048_RX_GPIO GPIO_NUM_14 // esp tx
#define M20048_TM_GPIO GPIO_NUM_17 // not used as of now
#define M20048_FIX_GPIO GPIO_NUM_16
#define M20048_HW_R_GPIO GPIO_NUM_18 // not used as of now
#define M20048_HW_S_GPIO GPIO_NUM_13 // not used as of now

// Though not mandatory, an event queue could be useful, so i'll add one.
static QueueHandle_t uart_queue; // handle for event queue (uart_event_t)

esp_err_t gps_init()
{
    // Init UART
    // I think a TX buffer is not needed since there is not that much traffic going from mcu to gps.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, UART_QUEUE_SIZE, &uart_queue, 0));

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

    return ESP_OK;
}

void gps_read() {
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