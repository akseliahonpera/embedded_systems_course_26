#include "imu.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "bno085_port.h"

static const char *TAG = "IMU";

/* These are the ESP32-S3 GPIOs connected to the BNO085 */
#define BNO085_HINT_GPIO  GPIO_NUM_5
#define BNO085_NRST_GPIO  GPIO_NUM_6
#define BNO085_BOOTN_GPIO GPIO_NUM_7

#define BNO085_REPORT_INTERVAL_US 10000U /* 100 Hz */

static TaskHandle_t imu_task_handle;
static volatile bool imu_reset_complete;

static void IRAM_ATTR hint_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imu_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;

    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }

    if (value.sensorId == SH2_ROTATION_VECTOR) {
        ESP_LOGI(TAG, "Rotation vector: r=%.3f i=%.3f j=%.3f k=%.3f",
                 value.un.rotationVector.real,
                 value.un.rotationVector.i,
                 value.un.rotationVector.j,
                 value.un.rotationVector.k);
    }
}

static void sh2_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET) {
        imu_reset_complete = true;
        ESP_LOGI(TAG, "BNO085 reset complete");
    }
}

static void imu_task(void *arg)
{
    (void)arg;

    for (;;) {
        /* HINT is active low.  The timeout also handles a missed edge. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        while (bno085_port_data_ready()) {
            sh2_service();
        }
    }
}

esp_err_t imu_init(i2c_master_bus_handle_t i2c_handle)
{
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
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed (%d)", rc);
        return ESP_FAIL;
    }
    if (!imu_reset_complete) {
        ESP_LOGE(TAG, "No reset advertisement from BNO085; check I2C, HINT, and SA0");
        sh2_close();
        return ESP_ERR_TIMEOUT;
    }

    rc = sh2_setSensorCallback(sensor_event_handler, NULL);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Could not register sensor callback (%d)", rc);
        return ESP_FAIL;
    }

    const sh2_SensorConfig_t rotation_vector_config = {
        .reportInterval_us = BNO085_REPORT_INTERVAL_US,
    };
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &rotation_vector_config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Could not enable rotation vector (%d)", rc);
        return ESP_FAIL;
    }

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BNO085_HINT_GPIO, hint_isr_handler, NULL),
                        TAG, "Could not install BNO085 interrupt handler");

    BaseType_t task_created = xTaskCreate(imu_task, "bno085", 4096, NULL,
                                          5, &imu_task_handle);
    if (task_created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* Do not wait for another edge if a packet is already pending. */
    if (bno085_port_data_ready()) {
        xTaskNotifyGive(imu_task_handle);
    }

    ESP_LOGI(TAG, "BNO085 initialized; rotation vector enabled at 100 Hz");
    return ESP_OK;
}
