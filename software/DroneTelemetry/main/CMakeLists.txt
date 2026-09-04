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

    INCLUDE_DIRS
        "."
        "barometer"
        "imu"
    REQUIRES
        driver
        esp_timer
        esp_driver_i2c
        esp_driver_gpio
)
