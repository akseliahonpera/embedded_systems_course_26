#pragma once

#include <stdint.h>
#include <assert.h>

#define CMD_ACT                 (1UL << 0)
#define CMD_PING                (1UL << 1)

#define CMD_DISABLE_IMU         (1UL << 2)
#define CMD_ENABLE_IMU          (1UL << 3)
#define CMD_RESET_IMU           (1UL << 4)
#define CMD_IMU_MEASURE_COV     (1UL << 5)
#define CMD_IMU_MEASURE_BIAS    (1UL << 6)

#define CMD_DISABLE_GPS         (1UL << 7)
#define CMD_ENABLE_GPS          (1UL << 8)
#define CMD_RESET_GPS           (1UL << 9)

#define CMD_DISABLE_BAROMETER   (1UL << 10)
#define CMD_ENABLE_BAROMETER    (1UL << 11)
#define CMD_RESET_BAROMETER     (1UL << 12)

#define CMD_FUSION_START        (1UL << 13)
#define CMD_FUSION_STOP         (1UL << 14)

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

