#include "fusion.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "types.h"

static QueueHandle_t fusion_queue;
static QueueHandle_t telemetry_queue;

static const char *TAG = "FUSION";

static void fusion_task(void *arg)
{
    (void)arg;

    sensor_msg_t msg;
    fusion_msg_t msg_out;

    imu_data_t imu_data_newest;
    gps_data_t gps_data_newest;
    baro_data_t barometer_data_newest;

    while (1)
    {
        // Sleep until there is data in the queue.
        if (xQueueReceive(fusion_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            // proc message
            // fuusioi anturidata, lopuksi lähetä data telemetriataskille
            switch (msg.type)
            {
            case SENSOR_BARO:
                break;

            case SENSOR_IMU:
                /* code */
                break;

            case SENSOR_GPS:
                /* code */
                break;

            default:
                break;
            }
            xQueueSend(telemetry_queue, &msg_out, 0); // 0 tarkoittaa että tämä funktio palaa heti jos jono on täysi.
        }
    }
}

esp_err_t fusion_init(QueueHandle_t fusion_queue_handle, QueueHandle_t telemetry_queue_handle)
{
    fusion_queue = fusion_queue_handle;
    telemetry_queue = telemetry_queue_handle;

    TaskHandle_t fusion_task_handle;

    BaseType_t task_created = xTaskCreate(fusion_task, "FUSION_TASK", 4096, NULL,
                                          5, &fusion_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Fusion task started.");
    return ESP_OK;
}
