#include "fusion.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "types.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "fusion.h"

#define WGS84_A 6378137.0
#define WGS84_E2 0.00669437999014
#define M_PI 3.14159265358979323846
#define FUSION_STATUS_INTERVAL_US 1000000LL // One second.

/* The BNO085 linear-acceleration output still contains small attitude and
 * bias errors.  Do not allow a calibration performed while stationary to
 * claim that the navigation acceleration is more accurate than this. */
#define MIN_HORIZONTAL_ACCEL_VARIANCE 0.25f
#define MAX_PREDICTION_DT_S 0.100f
#define STATIONARY_ACCELERATION_MPS2 0.15f
#define STATIONARY_SAMPLE_COUNT 20U
#define GPS_POSITION_STDDEV_M 8.0f
#define GPS_INNOVATION_GATE 9.21f /* 99% chi-square gate, two position axes */

static QueueHandle_t telemetry_queue;
static TaskHandle_t fusion_task_handle;

static QueueHandle_t imu_queue;
static QueueHandle_t gps_queue;
static QueueHandle_t baro_queue;

static const char *TAG = "FUSION";

void fusion_send_data(const sensor_msg_t *msg)
{
    switch (msg->type)
    {
    case SENSOR_BARO:
        xQueueOverwrite(baro_queue, msg);
        break;
    case SENSOR_GPS:
        xQueueOverwrite(gps_queue, msg);
        break;
    case SENSOR_IMU:
        xQueueOverwrite(imu_queue, msg);
        break;
    default:
        break;
    }
    fusion_notify(FUSION_SENSOR_DATA_AVAILABLE);
}

void fusion_notify(fusion_command_t cmd)
{
    if (fusion_task_handle == NULL)
    {
        ESP_LOGW(TAG, "Fusion task is not initialized");
        return;
    }

    xTaskNotify(fusion_task_handle, cmd, eSetBits);
}

static double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

static void enu_reference_init(enu_reference_t *ref,
                               double lat0, double lon0, double alt0)
{
    const double lat0_rad = deg_to_rad(lat0);

    ref->lon0_rad = deg_to_rad(lon0);
    ref->sin_lat0 = sin(lat0_rad);
    ref->cos_lat0 = cos(lat0_rad);

    const double N0 = WGS84_A /
                      sqrt(1.0 - WGS84_E2 * ref->sin_lat0 * ref->sin_lat0);

    ref->r0 = (N0 + alt0) * ref->cos_lat0;
    ref->z0 = (N0 * (1.0 - WGS84_E2) + alt0) * ref->sin_lat0;
}

static void wgs84_to_enu(const enu_reference_t *ref,
                         double lat, double lon, double alt,
                         double *e, double *n, double *u)
{
    const double lat_rad = deg_to_rad(lat);
    const double dlon = deg_to_rad(lon) - ref->lon0_rad;

    const double sin_lat = sin(lat_rad);
    const double cos_lat = cos(lat_rad);
    const double sin_dlon = sin(dlon);
    const double cos_dlon = cos(dlon);

    const double N = WGS84_A /
                     sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    const double r = (N + alt) * cos_lat;
    const double z = (N * (1.0 - WGS84_E2) + alt) * sin_lat;

    const double dr = r * cos_dlon - ref->r0;
    const double dz = z - ref->z0;

    *e = r * sin_dlon;
    *n = -ref->sin_lat0 * dr + ref->cos_lat0 * dz;
    *u = ref->cos_lat0 * dr + ref->sin_lat0 * dz;
}

static void predict_phase(kalman_state *state, const sensor_msg_t *imu_data, float P[6][6], const float sigma[3][3])
{

    if (state->mode != TRACKING || imu_data->timestamp <= state->last_update)
        return;

    // Predict state
    float acc_east = imu_data->data.imu.data.acceleration_data.acc_x;
    float acc_north = imu_data->data.imu.data.acceleration_data.acc_y;
    const float dt = (1.0e-6) * (imu_data->timestamp - state->last_update);
    if (!isfinite(dt) || dt <= 0.0f || dt > MAX_PREDICTION_DT_S)
    {
        /* Queue overruns or a debugger pause must not turn a tiny bias into
         * seconds of fictitious motion.  Resume integration from this sample. */
        state->last_update = imu_data->timestamp;
        return;
    }
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    state->position_east += (state->velocity_east * dt) + (0.5f * dt2 * acc_east);
    state->position_north += (state->velocity_north * dt) + (0.5f * dt2 * acc_north);
    state->velocity_east += dt * acc_east;
    state->velocity_north += dt * acc_north;
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
    const float velocity_noise_scale = dt2;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            const float cov = i == j
                                  ? fmaxf(MIN_HORIZONTAL_ACCEL_VARIANCE, sigma[i][j])
                                  : sigma[i][j];

            P[i][j] += position_noise_scale * cov;

            P[i][j + 3] += cross_noise_scale * cov;

            P[i + 3][j] += cross_noise_scale * cov;

            P[i + 3][j + 3] += velocity_noise_scale * cov;
        }
    }
}

static void apply_stationary_constraint(kalman_state *state, float P[6][6])
{
    state->velocity_east = 0.0f;
    state->velocity_north = 0.0f;

    /* A zero-velocity observation also removes position/velocity correlation.
     * Keeping it would let a later GPS correction recreate velocity while the
     * unit is known to be at rest. */
    for (int i = 0; i < 6; ++i)
    {
        P[3][i] = P[i][3] = 0.0f;
        P[4][i] = P[i][4] = 0.0f;
    }
    P[3][3] = 0.01f;
    P[4][4] = 0.01f;
}

static void correction_phase_using_gps(kalman_state *state, const sensor_msg_t *gps_data, float P[6][6], const float R[2])
{

    const gps_data_t *gps = &gps_data->data.gps;
    if (!gps->has_fix)
        return;

    double gps_east, gps_north, gps_up;

    wgs84_to_enu(&state->origin, gps->latitude, gps->longtitude, 0, &gps_east, &gps_north, &gps_up);

    float S[2][2] = {
        {P[0][0], P[0][1]},
        {P[1][0], P[1][1]}};

    S[0][0] += R[0];
    S[1][1] += R[1];

    const float determinant = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (!isfinite(determinant) || determinant <= 1.0e-9f)
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

    const float innovation[2] = {
        gps_east - state->position_east,
        gps_north - state->position_north};

    const float nis = innovation[0] * (S[0][0] * innovation[0] + S[0][1] * innovation[1]) +
                      innovation[1] * (S[1][0] * innovation[0] + S[1][1] * innovation[1]);
    if (!isfinite(nis) || nis > GPS_INNOVATION_GATE)
    {
        ESP_LOGW(TAG, "Ignoring GPS outlier: innovation E=%.1f N=%.1f m (NIS=%.1f)",
                 innovation[0], innovation[1], nis);
        return;
    }

    state->position_east += innovation[0] * K[0][0] + innovation[1] * K[0][1];
    state->position_north += innovation[0] * K[1][0] + innovation[1] * K[1][1];
    state->position_up += innovation[0] * K[2][0] + innovation[1] * K[2][1];
    state->velocity_east += innovation[0] * K[3][0] + innovation[1] * K[3][1];
    state->velocity_north += innovation[0] * K[4][0] + innovation[1] * K[4][1];
    state->velocity_up += innovation[0] * K[5][0] + innovation[1] * K[5][1];

    /* Joseph form avoids the loss of symmetry/positive-semidefiniteness that
     * the simplified covariance update suffers from in single precision. */
    float A[6][6] = {0};
    float AP[6][6] = {0};
    float updated_P[6][6] = {0};
    for (int row = 0; row < 6; ++row)
    {
        A[row][row] = 1.0f;
        A[row][0] -= K[row][0];
        A[row][1] -= K[row][1];
    }
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
            for (int k = 0; k < 6; ++k)
                AP[row][col] += A[row][k] * P[k][col];

    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
        {
            for (int k = 0; k < 6; ++k)
                updated_P[row][col] += AP[row][k] * A[col][k];
            updated_P[row][col] += K[row][0] * R[0] * K[col][0] +
                                   K[row][1] * R[1] * K[col][1];
        }
    memcpy(P, updated_P, sizeof(updated_P));
}

static void init_kalman_filter(kalman_state *state, const sensor_msg_t *gps_data)
{

    if (gps_data->data.gps.has_fix)
    {
        state->gps_fix_counter++;
        state->init_latitude_sum += gps_data->data.gps.latitude;
        state->init_longitude_sum += gps_data->data.gps.longtitude;
    }
    else
    {
        state->gps_fix_counter = 0;
        state->init_latitude_sum = 0.0;
        state->init_longitude_sum = 0.0;
    }

    if (state->gps_fix_counter >= 3)
    {
        const double origin_latitude = state->init_latitude_sum / state->gps_fix_counter;
        const double origin_longitude = state->init_longitude_sum / state->gps_fix_counter;
        *state = (kalman_state){0};
        enu_reference_init(&state->origin, origin_latitude, origin_longitude, 0.0);
        state->last_update = gps_data->timestamp;
        state->mode = TRACKING;
        ESP_LOGI(TAG, "Kalman filter tracking; averaged GPS origin lat=%.7f lon=%.7f", origin_latitude, origin_longitude);
    }
}

static void reset_state_covariance(float covariance[6][6])
{
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            covariance[row][col] = row == col ? (row < 3 ? 9.0f : 1.0f) : 0.0f;
        }
    }
}

static bool gps_data_check(const sensor_msg_t *msg)
{
    const gps_data_t *gps = &msg->data.gps;

    return gps->has_fix &&
           isfinite(gps->latitude) &&
           isfinite(gps->longtitude) &&
           fabsf(gps->latitude) <= 90.0f &&
           fabsf(gps->longtitude) <= 180.0f;
}

static void print_kalman_state(const kalman_state *state, const char *covariance_source)
{
    const char *mode = state->mode == IDLE ? "IDLE" : state->mode == INIT ? "INIT (waiting for GPS)"
                                                                          : "TRACKING";
    ESP_LOGI(TAG,
             "\n\n"
             "  KALMAN STATUS  |  %s\n"
             "  ----------------------------------------------------------\n"
             "  %-18s %12s %12s %12s\n"
             "  %-18s %12.3f %12.3f %12.3f\n"
             "  %-18s %12.3f %12.3f %12.3f\n"
             "\n"
             "  IMU covariance source: %s\n"
             "  ----------------------------------------------------------\n",
             mode,
             "", "East", "North", "Up",
             "Position [m]", state->position_east, state->position_north, state->position_up,
             "Velocity [m/s]", state->velocity_east, state->velocity_north, state->velocity_up,
             covariance_source);
    fflush(stdout);
    fsync(fileno(stdout));
}

static void handle_fusion_commands(uint32_t events, kalman_state *state, float covariance[6][6],
                                   int64_t *start_timestamp, bool *periodic_status,
                                   int64_t *last_status_timestamp, bool *use_queue_covariance)
{

    if (events & FUSION_CMD_STOP)
    {
        state->mode = IDLE;
        ESP_LOGI(TAG, "Kalman filter stopped");
    }
    else if (events & FUSION_CMD_START)
    {
        *state = (kalman_state){.mode = INIT};
        reset_state_covariance(covariance);
        *start_timestamp = esp_timer_get_time();
        ESP_LOGI(TAG, "Kalman filter waiting for the next GPS fix");
    }

    if (events & FUSION_CMD_TOGGLE_PERIODIC_STATUS)
    {
        *periodic_status = !*periodic_status;
        *last_status_timestamp = esp_timer_get_time();
        ESP_LOGI(TAG, "Periodic Kalman status %s", *periodic_status ? "enabled" : "disabled");
    }

    if (events & FUSION_CMD_TOGGLE_IMU_COVARIANCE_SOURCE)
        *use_queue_covariance = !*use_queue_covariance;

    fflush(stdout);
    fsync(fileno(stdout));
}

static void fusion_task(void *arg)
{
    (void)arg;

    kalman_state state = {.gps_fix_counter = 0,
                          .mode = IDLE};
    int64_t start_timestamp = 0;
    bool periodic_status = false;
    int64_t last_status_timestamp = 0;

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
    float state_covariance[6][6];
    reset_state_covariance(state_covariance);

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
    const float manual_imu_covariance[3][3] = {
        {0.25f, 0.0f, 0.0f},
        {0.0f, 0.25f, 0.0},
        {0.0f, 0.0f, 0.25f}};
    float received_imu_covariance[3][3] = {0};
    bool has_imu_covariance = false;
    bool use_measured_covariance = true;

    // A standalone GPS fix is commonly noisier than a few metres, especially
    // indoors or with limited sky view.  Treat this as an 8 m 1-sigma fix.
    float gps_covariance[2] = {
        GPS_POSITION_STDDEV_M * GPS_POSITION_STDDEV_M,
        GPS_POSITION_STDDEV_M * GPS_POSITION_STDDEV_M};

    sensor_msg_t msg;

    while (1)
    {
        // A notification does not unblock xQueueReceive. Bound this wait so
        // commands are handled even when sensors are silent; keep draining
        // sensor messages while IDLE to avoid blocking their producers.
        uint32_t events = 0;

        if (xTaskNotifyWait(0, UINT32_MAX, &events, 100) == pdTRUE)
        {
            if (xQueueReceive(gps_queue, &msg, pdMS_TO_TICKS(0)))
            {

                if (!gps_data_check(&msg))
                {
                    if (state.mode == INIT)
                    {
                        state.gps_fix_counter = 0;
                        state.init_latitude_sum = 0.0;
                        state.init_longitude_sum = 0.0;
                    }
                }
                else if (state.mode == INIT)
                {
                    init_kalman_filter(&state, &msg);
                }
                else if (state.mode == TRACKING)
                {
                    correction_phase_using_gps(&state, &msg, state_covariance, gps_covariance);
                }
            }
            if (xQueueReceive(baro_queue, &msg, pdMS_TO_TICKS(0)))
            {
                // kalman_baro_update(&msg);
            }
            if (xQueueReceive(imu_queue, &msg, pdMS_TO_TICKS(0)))
            {
                if (msg.data.imu.data_type == COVARIANCE_DATA)
                {

                    received_imu_covariance[0][0] = msg.data.imu.data.covariance_data.cov_x;
                    received_imu_covariance[1][1] = msg.data.imu.data.covariance_data.cov_y;
                    received_imu_covariance[2][2] = msg.data.imu.data.covariance_data.cov_z;
                    has_imu_covariance = true;
                    ESP_LOGI(TAG, "IMU covariance received (%s)", use_measured_covariance ? "in use" : "stored; manual values in use");
                }
                else if (msg.data.imu.data_type == ACCELERATION_DATA && state.mode == TRACKING)
                {
                    const float (*imu_covariance)[3] = use_measured_covariance && has_imu_covariance ? received_imu_covariance : manual_imu_covariance;
                    predict_phase(&state, &msg, state_covariance, imu_covariance);

                    const float acceleration = hypotf(msg.data.imu.data.acceleration_data.acc_x,
                                                      msg.data.imu.data.acceleration_data.acc_y);
                    if (isfinite(acceleration) && acceleration < STATIONARY_ACCELERATION_MPS2)
                    {
                        if (state.stationary_samples < STATIONARY_SAMPLE_COUNT)
                            state.stationary_samples++;
                        if (state.stationary_samples == STATIONARY_SAMPLE_COUNT)
                            apply_stationary_constraint(&state, state_covariance);
                    }
                    else
                    {
                        state.stationary_samples = 0;
                    }
                }
            }
            handle_fusion_commands(events, &state, state_covariance, &start_timestamp,
                                   &periodic_status, &last_status_timestamp, &use_measured_covariance);
        }

        // TODO: populate a fusion_msg_t from the estimate before sending
        // telemetry. The previous placeholder sent uninitialized data.

        // Check even after a queue timeout, so output also works without sensors.
        const int64_t now = esp_timer_get_time();
        const bool periodic_due = periodic_status && now - last_status_timestamp >= FUSION_STATUS_INTERVAL_US;
        if ((events & (FUSION_CMD_PRINT_STATUS | FUSION_CMD_TOGGLE_IMU_COVARIANCE_SOURCE)) || periodic_due)
        {
            const char *source = !use_measured_covariance ? "manual" : has_imu_covariance ? "measured"
                                                                                          : "measured (manual fallback)";
            print_kalman_state(&state, source);
        }
        if (periodic_due)
        {
            // Skip missed intervals rather than printing a burst after a delay.
            last_status_timestamp = now;
        }
    }
}

esp_err_t fusion_init(QueueHandle_t telemetry_queue_handle)
{

    imu_queue = xQueueCreate(1, sizeof(sensor_msg_t));
    gps_queue = xQueueCreate(1, sizeof(sensor_msg_t));
    baro_queue = xQueueCreate(1, sizeof(sensor_msg_t));

    if (imu_queue == NULL || gps_queue==NULL || baro_queue==NULL || telemetry_queue_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    telemetry_queue = telemetry_queue_handle;

    BaseType_t task_created = xTaskCreate(fusion_task, "FUSION_TASK", 4096, NULL,
                                          5, &fusion_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Fusion task started.");
    return ESP_OK;
}
