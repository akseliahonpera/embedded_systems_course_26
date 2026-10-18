#pragma once

#include <stdint.h>


enum DeviceStatusFlag: uint32_t
{
    STATUS_GNSS_ENABLED = 1 << 0,
    STATUS_GNSS_HAS_FIX = 1 << 1
};

struct __attribute__((__packed__)) ControlDatagram
{
    int32_t command;
    int32_t packed_num;
};

struct  __attribute__((__packed__)) Datagram
{
    float location[2];
    float rotation[3];
    float velocity[3];
    float pressure;
    float temperature;

    uint32_t status;

    int32_t packet_num;
    float time;
};

enum ControlCommand: uint32_t
{
    CMD_ACT                 = 1 << 0,
    CMD_PING                = 1 << 1,
    CMD_CALIBRATE_IMU       = 1 << 2,
    CMD_CALIBRATE_COMPASS   = 1 << 3,
    CMD_CALIBRATE_BAROMETER = 1 << 4,
    CMD_DISABLE_GPS         = 1 << 5,
    CMD_ENABLE_GPS          = 1 << 6
};
