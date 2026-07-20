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

// initialization and reset
static void imu_hardware_reset(void)
{
    // Configure both control pins as standard outputs with active pull-ups
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NRST) | (1ULL << PIN_BOOTN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Keep internal pull-ups active during configuration
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_config_t hint_conf = {
        .pin_bit_mask = (1ULL << PIN_HINT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, 
    };
    gpio_config(&hint_conf);

    // 1. CRUCIAL STEP: Pull BOOTN high FIRST and wait for the pin voltage to settle
    gpio_set_level(PIN_BOOTN, 1); 
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Pulse Reset Low while keeping BOOTN strictly driven high
    gpio_set_level(PIN_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); // Hold reset low for 50ms
    gpio_set_level(PIN_NRST, 1);   // Release reset line

    // 3. Extended boot time window to let the app calibrate internal clocks
    vTaskDelay(pdMS_TO_TICKS(400)); 

    // Install ISR handlers now that the chip is awake
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_HINT, hint_isr_handler, NULL);
}


// service task
static void imu_service_task(void *pvParameters)
{
    imu_task_handle = xTaskGetCurrentTaskHandle();

    // Configure and start rotation vector reports at 100Hz (10,000 microseconds)
    sh2_SensorConfig_t config = {
        .changeSensitivityEnabled = false,
        .wakeupEnabled = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled = false,
        .changeSensitivity = 0,
        .reportInterval_us = 10000,
        .batchInterval_us = 0,
        .sensorSpecific = 0};

    int rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "Failed to enable Rotation Vector report: %d", rc);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "BNO085 streaming started.");

    while (1)
    {
        // Wait here passively until HINT pin goes low and notifies this task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Run the service engine to handle outstanding packets
        sh2_service();
    }
}

// External declarations matching your port layer
esp_err_t bno085_port_init(i2c_master_bus_handle_t bus, uint32_t i2c_freq_hz);
extern sh2_Hal_t bno085_hal;

/**
 * @brief High-level entry point called from main app.
 * @param shared_bus_handle The already initialized I2C master bus handle from main.c
 */
esp_err_t imu_init(i2c_master_bus_handle_t shared_bus_handle)
{
    // A. Check if the shared bus is valid
    if (shared_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "Passed I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // B. Initialize custom port layer to populate 'bno085_dev' using the shared bus
    esp_err_t err = bno085_port_init(shared_bus_handle, 100000);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BNO085 port device handle: %s", esp_err_to_name(err));
        return err;
    }

    // C. Put hardware into operational state (Configure pins, fire hardware reset)
    imu_hardware_reset();

    // D. Register the I2C HAL and open the SH2 connection
    int rc = sh2_open(&bno085_hal, sh2_event_handler, NULL);
    if (rc != SH2_OK)
    {
        ESP_LOGE(TAG, "Failed to open SH2 session: %d", rc);
        return ESP_FAIL;
    }

    // E. Register our sensor data callback
    sh2_setSensorCallback(sensor_event_handler, NULL);

    // F. Launch the task to process data continuously
    xTaskCreatePinnedToCore(
        imu_service_task,
        "imu_task",
        4096,
        NULL,
        10,
        NULL,
        1);

    return ESP_OK;
}
