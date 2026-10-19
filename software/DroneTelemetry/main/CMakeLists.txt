idf_component_register(
    SRCS
        "main.c"

        "barometer/barometer.c"
        "barometer/bmp5.c"
        "barometer/bmp581_port.c"
        "barometer/bmp5_selftest.c"

        "imu/sh2.c"
        "imu/bno085_port.c"
        "imu/imu.c"
        "imu/sh2_SensorValue.c"
        "imu/shtp.c"
        "imu/sh2_util.c"
        "imu/euler.c"

        "gps/gps.c"
        "gps/nmea_parser.c"

        "telemetry/telemetry.c"

        "fusion/fusion.c"

        "comm/telem_client.c"
        "comm/udp_client.c"
        "comm/wifi_sta.c"

    INCLUDE_DIRS
        "."
        "barometer"
        "imu"
        "gps"
        "telemetry"
        "fusion"
        "comm"
    REQUIRES
        driver
        esp_driver_i2c
        esp_driver_gpio
        esp_timer
        esp_event
        esp_driver_uart
        esp_wifi
        esp_netif
        nvs_flash

)
