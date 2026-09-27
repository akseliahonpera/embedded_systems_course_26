#include "fusion.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "types.h"
#include <math.h>


#define M_PI 3.14159265358979323846
#define WGS84_A 6378137.0  
#define WGS84_E2 0.00669437999014

static QueueHandle_t fusion_queue;
static QueueHandle_t telemetry_queue;

static const char *TAG = "FUSION";



static double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}


static void adjust_for_declination(float* adjusted_x, float* adjusted_y, const float original_x, const float original_y) {
    const float declination_rad = (12.64 * M_PI) / 180.0;

    const float cosine = cosf(declination_rad);
    const float sine = sinf(declination_rad);

    *adjusted_x = cosine * original_x +  sine * original_y;
    *adjusted_y = -sine * original_x +  cosine * original_y;
}


static void predict_phase(kalman_state* state, const sensor_msg_t* imu_data, const float P[6][6], const float sigma[3]){

    if (!state->init ||imu_data->timestamp <= state->last_update) return;

    // Predict state
    float acc_east, acc_north;
    adjust_for_declination(&acc_east, &acc_north, imu_data->data.imu.data.acceleration_data.acc_x, imu_data->data.imu.data.acceleration_data.acc_y);
    float dt = (1.0e-6) * (imu_data->timestamp - state->last_update);
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt; 


    
    float next_position_east = state->position_east + (state->velocity_east * dt) + (0.5f * dt2 * acc_east);
    float next_position_north = state->position_north + (state->velocity_north * dt) + (0.5f * dt2 * acc_north);
    float next_velocity_east = state->velocity_east + dt * acc_east;
    float next_velocity_north = state->velocity_north + dt * acc_north;
    // float next_up = 0;        // Otetaan sit joskus käyttöön
    // float next_velocity_up = 0;

    // Adjust state estimate error covariance

    float Q[6][6] = {0.0};
    Q[0][0] = 0.25 * dt4 * sigma[0];
    Q[0][3] = 0.5 * dt3 * sigma[0];
    Q[1][1] = 0.25 * dt4 * sigma[1];
    Q[1][4] = 0.5 * dt3 * sigma[1];
    Q[2][2] = 0.25 * dt4 * sigma[2];
    Q[2][5] = 0.5 * dt3 * sigma[2];
    Q[3][0] = 0.5 * dt3 * sigma[0];
    Q[3][3] = dt2 * sigma[0];
    Q[4][1] = 0.5 * dt3 * sigma[1];
    Q[4][4] = dt2 * sigma[1];
    Q[5][2] = 0.5 * dt3 * sigma[2];
    Q[5][5] = dt2 * sigma[2];


    // float P_11_next = P[1][1] + 2 * dt * P[1][4] + dt2 * P[4][4]; 





    state->position_east = next_position_east;
    state->position_north = next_position_north;
    state->velocity_east = next_velocity_east;
    state->velocity_north = next_velocity_north;
    state->last_update = imu_data->timestamp;

    



    
}


static void wgs84_to_ecef(double lat, double lon, double alt, double *x, double *y, double *z) {
    double rad_lat = deg_to_rad(lat);
    double rad_lon = deg_to_rad(lon);
    
    double sin_lat = sin(rad_lat);
    double cos_lat = cos(rad_lat);
    double sin_lon = sin(rad_lon);
    double cos_lon = cos(rad_lon);
    
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    
    *x = (N + alt) * cos_lat * cos_lon;
    *y = (N + alt) * cos_lat * sin_lon;
    *z = (N * (1.0 - WGS84_E2) + alt) * sin_lat;
}

static void ecef_to_enu(double x, double y, double z, 
                 double lat0, double lon0, double alt0, 
                 double *e, double *n, double *u) {
    double x0, y0, z0;
    
    wgs84_to_ecef(lat0, lon0, alt0, &x0, &y0, &z0);
    
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;
    
    double rad_lat0 = deg_to_rad(lat0);
    double rad_lon0 = deg_to_rad(lon0);
    
    double sin_lat0 = sin(rad_lat0);
    double cos_lat0 = cos(rad_lat0);
    double sin_lon0 = sin(rad_lon0);
    double cos_lon0 = cos(rad_lon0);
    
    *e = -sin_lon0 * dx + cos_lon0 * dy;
    *n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz;
    *u =  cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz;
}



static void fusion_task(void *arg)
{
    (void)arg;


    kalman_state state;
    state.position_east = 0;
    state.position_north = 0;
    state.position_up = 0;
    state.velocity_east = 0;
    state.velocity_north = 0;
    state.velocity_up = 0;
    state.init = false;


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
                if (msg.data.imu.data_type == ACCELERATION_DATA) {
                    
                }
                break;

            case SENSOR_GPS:
                
                if (msg.data.gps.has_fix) {

                }
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
