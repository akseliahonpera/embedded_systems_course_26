idf_component_register(
    SRCS
        "main.c"
        "barometer/barometer.c"
        "barometer/bmp5.c"
        "barometer/bmp581_port.c"
        "barometer/bmp5_selftest.c"

    INCLUDE_DIRS
        "."
        "barometer"
        "gps"
        "imu"
)