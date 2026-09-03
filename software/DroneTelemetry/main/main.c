#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "barometer.h"
#include "imu.h"
#include "gps.h"
#include "esp_log.h"
#include "types.h"
static const char *TAG = "MAIN";

// I2C confs
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_GLITCH_IGNORE_COUNT 7
#define I2C_FREQ_HZ 100000

void app_main(void)
{
    /*
    Mun idea ois, että kapseloidaan kaikki mitä mahdollista omiin tiedostoihin.
    Eli kutsuttas vain kunkin anturin init funktiota mainista.
    Jaettu I2C on järkevää alustaa täällä, koska se täytyy passata barometrille ja imulle,
    mutta UART on point-to-point joten se on sama alustaa gps.c:ssä.
    
    Sitten voitas käyttää jonoa näiden anturitaskien ja fuusiotaskin kommunikointiin:

        static QueueHandle_t sensor_fusion_queue;

    Periaatteessa vois olla useampikin jono, mutta mun mielipide on, että yhellä jonolla 
    toteutus on siistimpi, koska fuusiotaskin tarttee tällöin herätä ainoastaan yhden jonon dataan:
        
        while(1) {
            if (xQueueReceive(sensor_fusion_queue, &data, 0) == pdTRUE) {
            handle(&data);
            }   
        }

    Tätä varten täytyy vaan määritellä jonkinnäköset "geneeriset" tietorakenteet, jotta data saadaan parsittua
    ja käsiteltyä oikein. Esim jotain tänkaltasta:

        Anturin tyyppi. Käytetään fuusiotaskin switchissä jotta datalle saadaan tehtyä oikeat temput
        typedef enum {
            SENSOR_BARO
            SENSOR_IMU
            SENSOR_GPS
        } sensor_type_t;

        Datatyypit. Näille määritellään sopivat structit
        typedef union {
            imu_data_t imu;
            baro_data_t baro;
            gps_data_t gps;
        } sensor_payload_t;

        Viesti joka pukataan jonoon
        typedef struct {
            sensor_type_t type;
            int timestamp;
            sensor_payload_t data;
        } sensor_message_t;

    Eli homma toimis suurinpiirtein näin:
    gps_task ---------|
    imu_task* --------|---> QueueHandle_t sensor_fusion_queue --> sensor_fusion_task --> jono? --> datanlähetystaski
    barometer_task ---|
    *POIKKEUS: imun sh2-kirjasto käyttää callback-funktiota (sensor_event_handler). Tämä callback saa puskea datan jonoon itse.
   
    */

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

    vTaskDelay(pdMS_TO_TICKS(100));

    // /* Initialize the barometer first */
    // ESP_LOGI(TAG, "Initializing Barometer...");
    // ESP_ERROR_CHECK(barometer_init(i2c_bus, I2C_FREQ_HZ));
    // ESP_LOGI(TAG, "Barometer initialized successfully.");


    vTaskDelay(pdMS_TO_TICKS(100));

    
    // ESP_LOGI(TAG, "Initializing IMU...");
    // esp_err_t imu_err = imu_init(i2c_bus);
    // if (imu_err == ESP_OK)
    // {
    //     ESP_LOGI(TAG, "IMU initialized successfully.");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "IMU unavailable (%s)", esp_err_to_name(imu_err));
    // }

    gps_init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    barometer_init(i2c_bus, I2C_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_init(i2c_bus);


}
