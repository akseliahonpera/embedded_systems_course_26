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


static void predict_phase(kalman_state* state, const sensor_msg_t* imu_data, float P[6][6], const float sigma[3]){

    if (!state->init || imu_data->timestamp <= state->last_update) return;

    // Predict state
    float acc_east, acc_north;
    adjust_for_declination(&acc_east, &acc_north, imu_data->data.imu.data.acceleration_data.acc_x, imu_data->data.imu.data.acceleration_data.acc_y);
    const float dt = (1.0e-6) * (imu_data->timestamp - state->last_update);
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    state->position_east  +=  (state->velocity_east * dt) + (0.5f * dt2 * acc_east);
    state->position_north += (state->velocity_north * dt) + (0.5f * dt2 * acc_north);
    state->velocity_east  +=  dt * acc_east;
    state->velocity_north +=  dt * acc_north;
    state->last_update = imu_data->timestamp;

    // up suunta toistaseksi pois käytöstä eli
    // sijainti/nopeus arviointia ei päivitetä sille, mutta
    // alempana kuitenkin lasketaan noin ko-/varianssit jos halutaan laajentaa
    // 3D:hen.


    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            const float pv = P[i][j + 3];
            const float vp = P[i + 3][j];
            const float dt_vv = dt * P[i + 3][j + 3];

            P[i][j] += dt * (pv + vp + dt_vv);
            P[i][j + 3] = pv + dt_vv;
            P[i + 3][j] = vp + dt_vv;
        }
    }

    
    const float position_noise_scale = 0.25f * dt4;
    const float cross_noise_scale = 0.5f * dt3;
    const float cross_noise_0 = cross_noise_scale * sigma[0];
    P[0][0] += position_noise_scale * sigma[0];
    P[0][3] += cross_noise_0;
    P[3][0] += cross_noise_0;
    P[3][3] += dt2 * sigma[0];

    const float cross_noise_1 = cross_noise_scale * sigma[1];
    P[1][1] += position_noise_scale * sigma[1];
    P[1][4] += cross_noise_1;
    P[4][1] += cross_noise_1;
    P[4][4] += dt2 * sigma[1];

    const float cross_noise_2 = cross_noise_scale * sigma[2];
    P[2][2] += position_noise_scale * sigma[2];
    P[2][5] += cross_noise_2;
    P[5][2] += cross_noise_2;
    P[5][5] += dt2 * sigma[2];


    
}


static void correction_phase_using_gps(kalman_state *state, const sensor_msg_t *gps_data, float P[6][6], const float R[2])
{

    const gps_data_t *gps = &gps_data->data.gps;
    if (!gps->has_fix)
        return;

    float x, y, z;
    float gps_east, gps_north, gps_up;

    wgs84_to_ecef(gps->latitude, gps->longtitude, state->origo_altitude, &x, &y, &z);
    ecef_to_enu(x, y, z, state->origo_latitude, state->origo_longitude, state->origo_altitude, &gps_east, &gps_north, &gps_up);

    float S[2][2] = {
        {P[0][0], P[0][1]},
        {P[1][0], P[1][1]}};

    S[0][0] += R[0];
    S[1][1] += R[1];

    const float determinant = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (determinant == 0.0f)
        return;

    const float inverse_determinant = 1.0f / determinant;
    const float s00 = S[0][0];
    S[0][0] = S[1][1] * inverse_determinant;
    S[0][1] *= -inverse_determinant;
    S[1][0] *= -inverse_determinant;
    S[1][1] = s00 * inverse_determinant;

    float K[6][2];
    for (int i = 0; i < 6; ++i)
    {
        K[i][0] = P[i][0] * S[0][0] + P[i][1] * S[1][0];
        K[i][1] = P[i][0] * S[0][1] + P[i][1] * S[1][1];
    }

    


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

    /*
     [ σE²    0     0      0      0      0   ]
     [  0    σN²    0      0      0      0   ]
     [  0     0    σU²     0      0      0   ]
     [  0     0     0     σvE²    0      0   ]
     [  0     0     0      0     σvN²    0   ]
     [  0     0     0      0      0     σvU² ]
    */
   // Nää on hatusta, tässä 9.0 tarkoittaa että alun sijainnin keskihajonta 3m
   // ja 1.0 tarkoittaa että alkunopeuden kekihajonta 1m/s. Käytännössä siis arvot alkutilan epävarmuudelle.
    float state_covariance[6][6] = {
        {9.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 9.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 9.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

        

    /*
     [ σx²    0     0  ]
     [  0    σy²    0  ]
     [  0     0    σz² ]
    */
    // Tässä arvot vaan placeholderina
    // Datasheetissä σ=0.35 => σ^2 = 0.125 ois raa'alle kiihtyvyydelle,
    // mutta tässä on muutaki prosessointia.
    // Tää kannattaa alustaa kalibroinnilla, jossa mitataan
    // imun kiihtyvyysdatan kovarianssia levossa.
    float imu_covariance[3] = {
        0.25f, 0.25f, 0.25f};


    // Nääkin vois määrittää kalibroinnilla,
    // mutta vaatii vähän enemmän työtä.
    // Myös HDOP gps:ltä ois hyvä lisä epävarmuuden arvioinnissa 
    float gps_covariance[2] = {
        4.51f, 4.51f};

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
