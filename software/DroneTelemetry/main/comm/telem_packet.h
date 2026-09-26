#pragma once

#include <stdint.h>
#include <assert.h>


#define CMD_ACT                 (1 << 0)
#define CMD_PING                (1 << 1)
#define CMD_CALIBRATE_IMU       (1 << 2)
#define CMD_CALIBRATE_COMPASS   (1 << 3)
#define CMD_CALIBRATE_BAROMETER (1 << 4)
#define CMD_DISABLE_GPS         (1 << 5)
#define CMD_ENABLE_GPS          (1 << 6)


typedef struct __attribute__((packed)) {
    float location[2];
    float rotation[3];
    float velocity[3];
    float pressure;
    float temperature;

    uint32_t status;

    int32_t packet_num;
    float time;
} telem_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t command;
    int32_t packet_num;
} control_packet_t;

