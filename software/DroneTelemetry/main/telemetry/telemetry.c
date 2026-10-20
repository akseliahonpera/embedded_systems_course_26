#include "telemetry.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "types.h"
#include "telem_client.h"
#include "udp_client.h"
#include "telem_packet.h"

#include "imu.h"
#include "barometer.h"
#include "gps.h"
#include "fusion.h"

static QueueHandle_t telemetry_queue;

static udp_client_t udp_client;
static telem_client_t telem_client;

static const char *TAG = "TELEMETRY";

#define CLIENT_IP "PLACEHOLDER"
#define CLIENT_PORT 5000

static void map_packet(fusion_msg_t *fusion_data, telem_packet_t *telem_packet)
{
    telem_packet->location[0] = fusion_data->state.latitude;
    telem_packet->location[1] = fusion_data->state.longtitude;
    telem_packet->rotation[0] = fusion_data->state.roll;
    telem_packet->rotation[1] = fusion_data->state.pitch;
    telem_packet->rotation[2] = fusion_data->state.yaw;
    telem_packet->velocity[0] = fusion_data->state.velocity; // TODO Tää on vektori, pitääkattoa structit kuntoon
    telem_packet->pressure    = fusion_data->state.pressure;
    telem_packet->temperature = fusion_data->state.temperature;
    telem_packet->time        = fusion_data->timestamp;
    // Altitude? Relative position in meters?
}

static void telemetry_task(void *arg)
{
    (void)arg;

    fusion_msg_t msg;

    while (1)
    {
        // Proc pending commands
        int command = telem_client_poll_command(&telem_client);
        switch (command)
        {
        case CMD_PING:
            break;

        // IMU
        case CMD_DISABLE_IMU:
            imu_notify(IMU_CMD_STANDBY);
            break;
        case CMD_ENABLE_IMU:
            imu_notify(IMU_CMD_EXIT_STANDBY);
            break;
        case CMD_IMU_MEASURE_BIAS:
            imu_notify(IMU_CMD_START_BIAS_MEASUREMENT);
            break;
        case CMD_IMU_MEASURE_COV:
            imu_notify(IMU_CMD_START_COVARIANCE_MEASUREMENT);
            break;
        case CMD_CALIBRATE_IMU:
            // TODO
            break;
        case CMD_CALIBRATE_COMPASS:
            // TODO
            break;

        // BAROMETER
        case CMD_CALIBRATE_BAROMETER:
            // TODO
            break;

        case CMD_DISABLE_GPS:
            gps_notify(GPS_CMD_STANDBY);
            break;
        case CMD_ENABLE_GPS:
            gps_notify(GPS_CMD_EXIT_STANDBY);
            break;
        case CMD_RESET_GPS:
            gps_notify(GPS_CMD_RESET);
            break;

        case CMD_FUSION_STOP:
            fusion_notify(FUSION_CMD_STOP);
            break;
        case CMD_FUSION_START:
            fusion_notify(FUSION_CMD_START);
        default:
            break;
        }

        // Proc pending telemetry data
        if (xQueueReceive(telemetry_queue, &msg, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Mäppää fuusioitu anturidata telem_packet_t muotoon -> lähetä
            telem_packet_t packet = {0};
            map_packet(&msg, &packet);
            telem_client_send(&telem_client, &packet);
        }
    }
}

esp_err_t telemetry_init(QueueHandle_t telemetry_queue_handle)
{
    telemetry_queue = telemetry_queue_handle;
    TaskHandle_t telemetry_task_handle;

    // Init UDP socket client
    esp_err_t err = udp_client_open(
        &udp_client,
        CLIENT_IP,
        CLIENT_PORT);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open UDP client");
        return err;
    }

    // Init telemetry protocol client
    telem_client_init(&telem_client, &udp_client);

    BaseType_t task_created = xTaskCreate(telemetry_task, "TELEMETRY_TASK", 4096, NULL,
                                          5, &telemetry_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Telemetry task started.");
    return ESP_OK;
}
