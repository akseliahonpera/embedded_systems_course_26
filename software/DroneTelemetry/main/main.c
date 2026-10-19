#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "types.h"

#include "barometer.h"
#include "imu.h"
#include "gps.h"

#include "telemetry.h"
#include "fusion.h"

#include "wifi_sta.h"

static const char *TAG = "MAIN";

// I2C confs
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000

#define WIFI_SSID       "PLACEHOLDER"
#define WIFI_PASSWORD   "PLACEHOLDER"
#define WIFI_TIMEOUT    100

static void print_menu(void)
{
    printf("\nSensor and Kalman menu\n"
           "---------------------------------------------------------------\n"
           "  B - Measure acceleration bias (keep the sensor stationary)\n"
           "  V - Measure acceleration variance (keep the sensor stationary)\n"
           "  K - Start/reinitialize Kalman filter from the next GPS fix\n"
           "  S - Stop Kalman filter\n"
           "  P - Print Kalman filter status\n"
           "  T - Toggle periodic Kalman filter status\n"
           "  C - Toggle IMU covariance source (manual / measured)\n"
           "---------------------------------------------------------------\n"
           "  1 - Exit GPS standby mode\n"
           "  2 - Set GPS to standby\n"
           "  3 - Perform GPS hardware reset\n"
           "  4 - Toggle GPS print\n"
           "---------------------------------------------------------------\n"
           "  M - Show this menu\n"
           "> ");
    fflush(stdout);
}

void app_main(void)
{
    // Initialize wifi
    wifi_sta_start(WIFI_SSID, WIFI_PASSWORD, WIFI_TIMEOUT);

    // Init I2C
    i2c_master_bus_handle_t i2c_bus = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_COUNT,
        .flags = {
            .enable_internal_pullup = false,
        }};

    // Initialize the shared I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // Initialize queues
    // Queue for fused data from sensorfusion task to telemetry task
    QueueHandle_t telemetry_queue = xQueueCreate(10, sizeof(fusion_msg_t));

    // Initialize fusion and telemetry tasks
    ESP_ERROR_CHECK(fusion_init(telemetry_queue));
    telemetry_init(telemetry_queue);

    // Initialize sensor tasks
    gps_init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    barometer_init(i2c_bus, I2C_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_err_t imu_result = imu_init(i2c_bus);
    if (imu_result != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU initialization failed: %s", esp_err_to_name(imu_result));
        return;
    }

    print_menu();
    for (;;)
    {
        int command = getchar();
        if (command == EOF)
        {
            // The console can return EOF when no input is available yet.
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        switch (command)
        {
        case 'b':
        case 'B':
            imu_notify(IMU_CMD_START_BIAS_MEASUREMENT);
            puts("\nBias measurement requested.");
            break;

        case 'v':
        case 'V':
            imu_notify(IMU_CMD_START_COVARIANCE_MEASUREMENT);
            puts("\nVariance measurement requested.");
            break;

        case 'k':
        case 'K':
            fusion_notify(FUSION_CMD_START);
            break;

        case 's':
        case 'S':
            fusion_notify(FUSION_CMD_STOP);
            break;

        case 'p':
        case 'P':
            fusion_notify(FUSION_CMD_PRINT_STATUS);
            break;

        case 'm':
        case 'M':
            print_menu();
            break;

        case 't':
        case 'T':
            fusion_notify(FUSION_CMD_TOGGLE_PERIODIC_STATUS);
            break;

        case 'c':
        case 'C':
            fusion_notify(FUSION_CMD_TOGGLE_IMU_COVARIANCE_SOURCE);
            break;

        case '1':
            gps_notify(GPS_CMD_EXIT_STANDBY);
            break;

        case '2':
            gps_notify(GPS_CMD_STANDBY);
            break;

        case '3':
            gps_notify(GPS_CMD_RESET);
            break;

        case '4':
            gps_notify(GPS_CMD_PRINT_STATUS);
            break;

        case '\r':
        case '\n':
        case ' ':
        case '\t':
            continue;

        default:
            puts("\nUnknown command. Use B, V, K, S, P, T, C, 1, 2, 3, 4 or M.");
            break;
        }

    }

    








}
