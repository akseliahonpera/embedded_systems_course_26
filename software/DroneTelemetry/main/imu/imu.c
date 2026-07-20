#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bno085_port.h"

#define PIN_HINT GPIO_NUM_5
#define PIN_NRST GPIO_NUM_6
#define PIN_BOOTN GPIO_NUM_7

static const char *TAG = "IMU_APP";
static TaskHandle_t imu_task_handle = NULL;
static volatile bool data_available = false;

// Host interrupt ISR
static void IRAM_ATTR hint_isr_handler(void *arg)
{
    data_available = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (imu_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Sensor event callback
static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;

    // Decode event
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK)
    {
        return;
    }

    if (value.sensorId == SH2_ROTATION_VECTOR)
    {
        ESP_LOGI(TAG, "Quaternion: [i: %.3f, j: %.3f, k: %.3f, r: %.3f]",
                 value.un.rotationVector.i,
                 value.un.rotationVector.j,
                 value.un.rotationVector.k,
                 value.un.rotationVector.real);
    }
}

static void sh2_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET)
    {
        ESP_LOGI(TAG, "BNO085 reset complete");
    }
}

