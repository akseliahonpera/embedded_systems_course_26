#pragma once

#include <stdint.h>


enum DeviceStatusFlag: uint32_t
{
    STATUS_GPS_ENABLED    = (1UL << 0),
    STATUS_GPS_HAS_FI     = (1UL << 1),
    STATUS_IMU_ENABLED    = (1UL << 2),
    STATUS_BARO_ENABLED   = (1UL << 3),
    STATUS_FUSION_ENABLED = (1UL << 4)
};

struct __attribute__((__packed__)) ControlDatagram
{
    int32_t command;
    int32_t packed_num;
};

struct  __attribute__((__packed__)) Datagram
{
    float coordinates[2];   // Latitude, Longtitude (deg)
    float location[3];      // East North Up (m)
    float rotation[4];      // Quaternion
    float velocity[3];      // East North Up (m/s)
    float pressure;         // (HPa)
    float temperature;      // (C)

    uint32_t status;

    int32_t packet_num;
    double time;
};

enum ControlCommand: uint32_t
{
    CMD_ACT                = (1UL << 0),
    CMD_PING               = (1UL << 1),

    CMD_DISABLE_IMU        = (1UL << 2),
    CMD_ENABLE_IMU         = (1UL << 3),
    CMD_RESET_IMU          = (1UL << 4),
    CMD_IMU_MEASURE_COV    = (1UL << 5),
    CMD_IMU_MEASURE_BIAS   = (1UL << 6),

    CMD_DISABLE_GPS        = (1UL << 7),
    CMD_ENABLE_GPS         = (1UL << 8),
    CMD_RESET_GPS          = (1UL << 9),

    CMD_DISABLE_BAROMETER  = (1UL << 10),
    CMD_ENABLE_BAROMETER   = (1UL << 11),
    CMD_RESET_BAROMETER    = (1UL << 12),

    CMD_FUSION_START       = (1UL << 13),
    CMD_FUSION_STOP        = (1UL << 14)

};
