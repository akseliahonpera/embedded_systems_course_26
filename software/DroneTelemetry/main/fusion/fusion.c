#include "fusion.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "types.h"


#define M_PI 3.14159265358979323846
#define WGS84_A 6378137.0  
#define WGS84_E2 0.00669437999014

static QueueHandle_t fusion_queue;
static QueueHandle_t telemetry_queue;

static const char *TAG = "FUSION";



static double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}


static void wgs84_to_ecef(float lat, float lon, float alt, float *x, float *y, float *z) {
    float rad_lat = deg_to_rad(lat);
    float rad_lon = deg_to_rad(lon);
    
    float sin_lat = sin(rad_lat);
    float cos_lat = cos(rad_lat);
    float sin_lon = sin(rad_lon);
    float cos_lon = cos(rad_lon);
    
    float N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    
    *x = (N + alt) * cos_lat * cos_lon;
    *y = (N + alt) * cos_lat * sin_lon;
    *z = (N * (1.0 - WGS84_E2) + alt) * sin_lat;
}

static void ecef_to_enu(float x, float y, float z, 
                 float lat0, float lon0, float alt0, 
                 float *e, float *n, float *u) {
    float x0, y0, z0;
    
    wgs84_to_ecef(lat0, lon0, alt0, &x0, &y0, &z0);
    
    float dx = x - x0;
    float dy = y - y0;
    float dz = z - z0;
    
    float rad_lat0 = deg_to_rad(lat0);
    float rad_lon0 = deg_to_rad(lon0);
    
    float sin_lat0 = sin(rad_lat0);
    float cos_lat0 = cos(rad_lat0);
    float sin_lon0 = sin(rad_lon0);
    float cos_lon0 = cos(rad_lon0);
    
    *e = -sin_lon0 * dx + cos_lon0 * dy;
    *n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz;
    *u =  cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz;
}



static void fusion_task(void *arg)
{
    (void)arg;


    kalman_state state;



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
