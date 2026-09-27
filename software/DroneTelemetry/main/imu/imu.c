#include "imu.h"

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

static void IRAM_ATTR hint_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (imu_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(imu_task_handle, &higher_priority_task_woken);
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

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;

    if (sh2_decodeSensorEvent(&value, event) != SH2_OK)
    {
        return;
    }

    sensor_msg_t msg;
    msg.type = SENSOR_IMU;
    msg.timestamp = esp_timer_get_time();

    static bool has_rotation_matrix = false;

    // Ideana myöhemmin siirtää rotaatiomatriisin laskenta fuusiotaskiin
    // ja poistaa kaikki paikalliset muuttujat täältä ja latoa arvot suoraan structeihin ja
    // siitä fusionqueen.
    static float r, i, j, k;
    static float acc_x, acc_y, acc_z, world_acc_x, world_acc_y, world_acc_z;
    static float roll_rad, pitch_rad, yaw_rad;
    static float rotation_matrix[3][3] = {0};

    switch (value.sensorId)
    {
    case SH2_ROTATION_VECTOR:
        r = value.un.rotationVector.real;
        i = value.un.rotationVector.i;
        j = value.un.rotationVector.j;
        k = value.un.rotationVector.k;

        calculate_rotation_matrix(r, i, j, k, rotation_matrix);
        has_rotation_matrix = true;

        q_to_ypr(r, i, j, k, &yaw_rad, &pitch_rad, &roll_rad);

        msg.data.imu.data_type = ROTATION_DATA;
        msg.data.imu.status = value.status;
        msg.data.imu.data.rotation_data.real = r;
        msg.data.imu.data.rotation_data.i = i;
        msg.data.imu.data.rotation_data.j = j;
        msg.data.imu.data.rotation_data.k = k;
        msg.data.imu.data.rotation_data.yaw_angle = yaw_rad * (180.0f / 3.14159265f);
        msg.data.imu.data.rotation_data.pitch_angle = pitch_rad * (180.0f / 3.14159265f);
        msg.data.imu.data.rotation_data.roll_angle = roll_rad * (180.0f / 3.14159265f);

        break;
    case SH2_LINEAR_ACCELERATION:
        acc_x = value.un.linearAcceleration.x;
        acc_y = value.un.linearAcceleration.y;
        acc_z = value.un.linearAcceleration.z;

        if (!has_rotation_matrix) return;

        world_acc_x = rotation_matrix[0][0] * acc_x + rotation_matrix[0][1] * acc_y + rotation_matrix[0][2] * acc_z;
        world_acc_y = rotation_matrix[1][0] * acc_x + rotation_matrix[1][1] * acc_y + rotation_matrix[1][2] * acc_z;
        world_acc_z = rotation_matrix[2][0] * acc_x + rotation_matrix[2][1] * acc_y + rotation_matrix[2][2] * acc_z;

        msg.data.imu.data_type = ACCELERATION_DATA;
        msg.data.imu.status = value.status;
        msg.data.imu.data.acceleration_data.acc_x = world_acc_x;
        msg.data.imu.data.acceleration_data.acc_y = world_acc_y;
        msg.data.imu.data.acceleration_data.acc_z = world_acc_z;

        break;
    default:
        return;
    }

    if (xQueueSend(fusion_queue, &msg, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Fusion queue full; IMU update discarded");
    }
    // static TickType_t last_print_time = 0;
    // const TickType_t print_interval = pdMS_TO_TICKS(500);
    // TickType_t current_time = xTaskGetTickCount();
    // if ((current_time - last_print_time) >= print_interval)
    // {
    //     float roll_deg = roll_rad * (180.0f / 3.14159265f);
    //     float pitch_deg = pitch_rad * (180.0f / 3.14159265f);
    //     float yaw_deg = yaw_rad * (180.0f / 3.14159265f);
    //     // ESP_LOGI(TAG, "Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°",
    //     //          roll_deg, pitch_deg, yaw_deg);
    //     ESP_LOGI(TAG, "acc_x=%.5f, acc_y=%.5f, acc_z=%.5f",
    //              world_acc_x, world_acc_y, world_acc_z);
    //     last_print_time = current_time;
    // }
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

    for (;;)
    {
        /* Wake periodically to report zero counts even if IMU reports stop. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
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
        xTaskNotifyGive(imu_task_handle);
    }

    ESP_LOGI(TAG, "IMU task started.");
    return ESP_OK;
}
