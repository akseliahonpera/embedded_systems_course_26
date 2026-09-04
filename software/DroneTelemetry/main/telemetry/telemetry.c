#include "telemetry.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "types.h"

static QueueHandle_t telemetry_queue;

static const char *TAG = "TELEMETRY";

static void telemetry_task(void *arg)
{
    (void)arg;

    fusion_msg_t msg;

    while (1) 
    {
        // Sleep until there is data in the queue.
        if (xQueueReceive(telemetry_queue, &msg, portMAX_DELAY) == pdTRUE) 
        {
            // proc message
            // lähetä data päätelaitteelle wifin välityksellä
            // ESP_LOGI(TAG, "Telemetry task received message");
        }
    }
}

esp_err_t telemetry_init(QueueHandle_t telemetry_queue_handle)
{
    telemetry_queue = telemetry_queue_handle;

    TaskHandle_t telemetry_task_handle;

    BaseType_t task_created = xTaskCreate(telemetry_task, "TELEMETRY_TASK", 4096, NULL,
                                          5, &telemetry_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Telemetry task started.");
    return ESP_OK;
}
