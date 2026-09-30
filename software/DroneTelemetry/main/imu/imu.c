#include "imu.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "bno085_port.h"

#include "types.h"
#include "euler.h"
#include "math.h"

#define M_PI 3.14159265358979323846
#define N 2500

static const char *TAG = "IMU";

/* These are the ESP32-S3 GPIOs connected to the BNO085 */
#define BNO085_HINT_GPIO GPIO_NUM_5
#define BNO085_NRST_GPIO GPIO_NUM_6
#define BNO085_BOOTN_GPIO GPIO_NUM_7

#define BNO085_REPORT_INTERVAL_US 10000U /* 100 Hz */

static TaskHandle_t imu_task_handle;
static volatile bool imu_reset_complete;

// Queue handle (to fusion task)
static QueueHandle_t fusion_queue;

// IMU state: currently used only during calibration of accelerometer bias
static imu_state_t imu_state = IMU_NORMAL;

void imu_notify(imu_command_t cmd)
{
    xTaskNotify(
        imu_task_handle,
        cmd,
        eSetBits);
}

static void IRAM_ATTR hint_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (imu_task_handle != NULL)
    {
        xTaskNotifyFromISR(
            imu_task_handle,
            IMU_DATA_READY,
            eSetBits,
            &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void calculate_rotation_matrix(float r, float i, float j, float k, float matrix[3][3])
{
    const float xx = i * i;
    const float yy = j * j;
    const float zz = k * k;
    const float xy = i * j;
    const float xz = i * k;
    const float yz = j * k;
    const float wx = r * i;
    const float wy = r * j;
    const float wz = r * k;

    matrix[0][0] = 1.0f - (2 * (yy + zz));
    matrix[0][1] = 2 * (xy - wz);
    matrix[0][2] = 2 * (xz + wy);

    matrix[1][0] = 2 * (xy + wz);
    matrix[1][1] = 1.0f - (2 * (xx + zz));
    matrix[1][2] = 2 * (yz - wx);

    matrix[2][0] = 2 * (xz - wy);
    matrix[2][1] = 2 * (yz + wx);
    matrix[2][2] = 1.0f - 2 * (xx + yy);
}

static void adjust_for_declination(float *adjusted_x, float *adjusted_y, const float original_x, const float original_y)
{
    const float declination_rad = (12.64 * M_PI) / 180.0;

    const float cosine = cosf(declination_rad);
    const float sine = sinf(declination_rad);

    *adjusted_x = cosine * original_x + sine * original_y;
    *adjusted_y = -sine * original_x + cosine * original_y;
}

static void send_message_to_fusion_queue(const sensor_msg_t *msg, bool force)
{

    const TickType_t wait_ticks = force ? portMAX_DELAY : 0;

    if (xQueueSend(fusion_queue, msg, wait_ticks) != pdTRUE)
    {
        ESP_LOGW(TAG, "Fusion queue full; IMU update discarded");
    }
}

static void handle_rotation_data(sh2_SensorValue_t *value, float rotation_matrix[3][3], bool *has_rotation_matrix, uint64_t *rotation_timestamp)
{
    sensor_msg_t msg = {0};

    const float r = value->un.rotationVector.real;
    const float i = value->un.rotationVector.i;
    const float j = value->un.rotationVector.j;
    const float k = value->un.rotationVector.k;

    calculate_rotation_matrix(r, i, j, k, rotation_matrix);
    *rotation_timestamp = value->timestamp;
    ;
    *has_rotation_matrix = true;

    msg.data.imu.data_type = ROTATION_DATA;

    msg.data.imu.data.rotation_data.real = r;
    msg.data.imu.data.rotation_data.i = i;
    msg.data.imu.data.rotation_data.j = j;
    msg.data.imu.data.rotation_data.k = k;

    msg.type = SENSOR_IMU;
    msg.data.imu.status = value->status;
    msg.data.imu.imu_internal_timestamp = value->timestamp;
    msg.data.imu.data.rotation_data.accuracy = value->un.rotationVector.accuracy;
    msg.timestamp = esp_timer_get_time();

    send_message_to_fusion_queue(&msg, false);
}

static void measure_acceleration_covariance(const sh2_SensorValue_t *value, float acc_x_world, float acc_y_world, float acc_z_world)
{
    static float mean[3] = {0.0f};
    static float m2[3] = {0.0f};
    static int count = 0;

    // Welford’s algorithm
    const float sample[3] = {
        acc_x_world,
        acc_y_world,
        acc_z_world};

    count++;

    if (count % 500 == 0)
    {
        ESP_LOGI(TAG, "Variance progress: %d/%d samples", count, N);
        fflush(stdout);          // Flush the C stream.
        fsync(fileno(stdout));   // Flush the USB transfer.
    }

    for (int i = 0; i < 3; i++)
    {
        float delta = sample[i] - mean[i];
        mean[i] += delta / count;
        float delta_after = sample[i] - mean[i];
        m2[i] += delta * delta_after;
    }

    if (count < N)
        return;

    sensor_msg_t cov_msg = {0};
    cov_msg.type = SENSOR_IMU;
    cov_msg.data.imu.data_type = COVARIANCE_DATA;
    cov_msg.data.imu.status = value->status;
    cov_msg.data.imu.imu_internal_timestamp = value->timestamp;
    cov_msg.timestamp = esp_timer_get_time();

    const float denominator = (float)(count - 1);
    cov_msg.data.imu.data.covariance_data.cov_x = m2[0] / denominator;
    cov_msg.data.imu.data.covariance_data.cov_y = m2[1] / denominator;
    cov_msg.data.imu.data.covariance_data.cov_z = m2[2] / denominator;

    send_message_to_fusion_queue(&cov_msg, true);

    for (int i = 0; i < 3; i++)
    {
        mean[i] = 0.0f;
        m2[i] = 0.0f;
    }

    count = 0;
    imu_state = IMU_NORMAL;
    ESP_LOGI(TAG, "Variance measurement ready");
    ESP_LOGI(TAG, "Variance: x=%.5f | y=%.5f | z=%.5f", cov_msg.data.imu.data.covariance_data.cov_x, cov_msg.data.imu.data.covariance_data.cov_y, cov_msg.data.imu.data.covariance_data.cov_z);
}

static void handle_acceleration_data(sh2_SensorValue_t *value, const float rotation_matrix[3][3], const bool has_rotation_matrix, const uint64_t *rotation_timestamp)
{

    static float bias[3] = {0.0f};
    static float sums[3] = {0.0f};
    static int count = 0;

    float acc_x = value->un.linearAcceleration.x;
    float acc_y = value->un.linearAcceleration.y;
    float acc_z = value->un.linearAcceleration.z;

    if (imu_state == IMU_MEASURING_BIAS)
    {
        sums[0] += acc_x;
        sums[1] += acc_y;
        sums[2] += acc_z;
        count++;

        if (count % 500 == 0)
        {
            ESP_LOGI(TAG, "Bias progress: %d/%d samples", count, N);
            fflush(stdout);          // Flush the C stream.
            fsync(fileno(stdout));   // Flush the USB transfer.
        }

        if (count >= N)
        {
            bias[0] = sums[0] / count;
            bias[1] = sums[1] / count;
            bias[2] = sums[2] / count;
            sums[0] = sums[1] = sums[2] = 0.0f;
            count = 0;
            imu_state = IMU_NORMAL;
            ESP_LOGI(TAG, "Bias measurement ready");
            ESP_LOGI(TAG, "Bias: x=%.5f | y=%.5f | z=%.5f", bias[0], bias[1], bias[2]);
        }
    }

    if (!has_rotation_matrix || *rotation_timestamp > value->timestamp || value->timestamp - *rotation_timestamp > 20000ULL)
    {
        return;
    }

    acc_x -= bias[0];
    acc_y -= bias[1];
    acc_z -= bias[2];

    float acc_x_world = rotation_matrix[0][0] * acc_x + rotation_matrix[0][1] * acc_y + rotation_matrix[0][2] * acc_z;
    float acc_y_world = rotation_matrix[1][0] * acc_x + rotation_matrix[1][1] * acc_y + rotation_matrix[1][2] * acc_z;
    float acc_z_world = rotation_matrix[2][0] * acc_x + rotation_matrix[2][1] * acc_y + rotation_matrix[2][2] * acc_z;
    adjust_for_declination(&acc_x_world, &acc_y_world, acc_x_world, acc_y_world);
    if (imu_state == IMU_MEASURING_COVARIANCE)
    {
        measure_acceleration_covariance(value, acc_x_world, acc_y_world, acc_z_world);
    }

    sensor_msg_t msg = {0};
    msg.type = SENSOR_IMU;
    msg.data.imu.data_type = ACCELERATION_DATA;
    msg.data.imu.data.acceleration_data.acc_x = acc_x_world;
    msg.data.imu.data.acceleration_data.acc_y = acc_y_world;
    msg.data.imu.data.acceleration_data.acc_z = acc_z_world;
    msg.data.imu.status = value->status;
    msg.data.imu.imu_internal_timestamp = value->timestamp;
    msg.timestamp = esp_timer_get_time();

    send_message_to_fusion_queue(&msg, false);
}

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{

    (void)cookie;
    sh2_SensorValue_t value;
    static bool has_rotation_matrix = false;
    static float rotation_matrix[3][3] = {0};
    static uint64_t rotation_timestamp = 0;

    if (sh2_decodeSensorEvent(&value, event) != SH2_OK)
    {
        return;
    }

    switch (value.sensorId)
    {
    case SH2_ROTATION_VECTOR:
        handle_rotation_data(&value, rotation_matrix, &has_rotation_matrix, &rotation_timestamp);
        break;

    case SH2_LINEAR_ACCELERATION:
        handle_acceleration_data(&value, rotation_matrix, has_rotation_matrix, &rotation_timestamp);
        break;
    default:
        return;
    }
}

static void sh2_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET)
    {
        imu_reset_complete = true;
        ESP_LOGI(TAG, "BNO085 reset complete");
    }
}

static void imu_task(void *arg)
{
    // This function waits for the HINT-activated ISR to wake it up, and notify the sh2 service to do it's thing.
    (void)arg;

    uint32_t events = 0;

    for (;;)
    {
        // /* Wake periodically to report zero counts even if IMU reports stop. */
        BaseType_t received = xTaskNotifyWait(
            0,
            UINT32_MAX,
            &events,
            pdMS_TO_TICKS(100));

        if (received == pdTRUE)
        {
            if (events & IMU_CMD_MODE_STANDBY)
            {
                // Set to standby here
            }

            if (events & IMU_CMD_MODE_NORMAL)
            {
                // Start IMU here
            }

            if (events & IMU_CMD_START_BIAS_MEASUREMENT)
            {
                switch (imu_state)
                {

                case IMU_NORMAL:
                    ESP_LOGI(TAG, "Starting bias measurement", NULL);
                    imu_state = IMU_MEASURING_BIAS;
                    break;

                case IMU_MEASURING_BIAS:
                    ESP_LOGI(TAG, "Already measuring bias", NULL);
                    break;

                default:
                    ESP_LOGI(TAG, "Can not measure bias now, state = %d", imu_state);
                    break;
                }
            }

            if (events & IMU_CMD_START_COVARIANCE_MEASUREMENT)
            {
                switch (imu_state)
                {

                case IMU_NORMAL:
                    ESP_LOGI(TAG, "Starting covariance measurement", NULL);
                    imu_state = IMU_MEASURING_COVARIANCE;
                    break;

                case IMU_MEASURING_COVARIANCE:
                    ESP_LOGI(TAG, "Already measuring covariance", NULL);
                    break;

                default:
                    ESP_LOGI(TAG, "Can not measure covariance now, state = %d", imu_state);
                    break;
                }
            }
        }

        while (bno085_port_data_ready())
        {
            sh2_service();
        }
    }
}

esp_err_t imu_init(QueueHandle_t fusion_queue_handle, i2c_master_bus_handle_t i2c_handle)
{
    // Obtain sensor fusion queue handle
    fusion_queue = fusion_queue_handle;

    ESP_RETURN_ON_ERROR(bno085_port_init(i2c_handle, BNO085_HINT_GPIO,
                                         BNO085_NRST_GPIO, BNO085_BOOTN_GPIO),
                        TAG, "BNO085 port initialization failed");

    /*
     * sh2_open() returns SH2_OK even when its advertise wait times out.
     * Do not issue a write until the BNO085 has actually advertised after
     * reset: on an absent or electrically stuck device that write can make
     * the ESP-IDF I2C driver hit a hardware-timeout panic.
     */
    imu_reset_complete = false;
    int rc = sh2_open(&bno085_hal, sh2_event_handler, NULL);
    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "sh2_open failed (%d)", rc);
        return ESP_FAIL;
    }
    if (!imu_reset_complete)
    {
        ESP_LOGE(TAG, "No reset advertisement from BNO085; check I2C, HINT, and SA0");
        sh2_close();
        return ESP_ERR_TIMEOUT;
    }

    rc = sh2_setSensorCallback(sensor_event_handler, NULL);
    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "Could not register sensor callback (%d)", rc);
        return ESP_FAIL;
    }

    const sh2_SensorConfig_t rotation_vector_config = {
        .reportInterval_us = BNO085_REPORT_INTERVAL_US,
    };
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &rotation_vector_config);

    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "Could not enable rotation vector (%d)", rc);
        return ESP_FAIL;
    }

    const sh2_SensorConfig_t linear_accelerometer_config = {
        .reportInterval_us = BNO085_REPORT_INTERVAL_US,
    };

    rc = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &linear_accelerometer_config);

    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "Could not enable linear accelerometer (%d)", rc);
        return ESP_FAIL;
    }

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BNO085_HINT_GPIO, hint_isr_handler, NULL),
                        TAG, "Could not install BNO085 interrupt handler");

    BaseType_t task_created = xTaskCreate(imu_task, "IMU_TASK", 4096, NULL,
                                          5, &imu_task_handle);
    if (task_created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    /* Do not wait for another edge if a packet is already pending. */
    if (bno085_port_data_ready())
    {
        xTaskNotify(imu_task_handle, IMU_DATA_READY, eSetBits);
    }

    ESP_LOGI(TAG, "IMU task started.");
    return ESP_OK;
}
