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
    float coordinates[2];   // Latitude, Longtitude (deg)
    float location[3];      // East North Up (m) 
    float rotation[4];      // Quaternion
    float velocity[3];      // East North Up (m/s)
    float pressure;         // (HPa)
    float temperature;      // (C)
    double time;            //

    uint32_t status;        // a bit vector for sensor states

    int32_t packet_num;
    
} telem_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t command;
    int32_t packet_num;
} control_packet_t;

typedef enum {
    STATUS_GPS_ENABLED    = (1UL << 0),
    STATUS_GPS_HAS_FIX    = (1UL << 1),
    STATUS_IMU_ENABLED    = (1UL << 2),
    STATUS_BARO_ENABLED   = (1UL << 3),
    STATUS_FUSION_ENABLED = (1UL << 4)
} device_status;