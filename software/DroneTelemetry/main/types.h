#ifndef TYPES_H
#define TYPES_H

#include "bmp5_defs.h" 
#include "sh2_SensorValue.h"
#include <stdbool.h>
#include <stdint.h> 

//TODO: En tiiä tartteeko näitä, voi käyttää periaatteessa valmiitakin tietorakenteita muista headereista
// Tuota sh2_SensorValue_t:tä vois varmaa nvähän siistiä
//-------------------------------------------------------------------------------------------------
// Custom lightweight data structures:

// Struct for barometer, derived from bmp5_defs.h
typedef struct
{
    float pressure;     // pressure (Pa)
    float temperature;  // temperature (°C)
} baro_data_t;

// Struct for imu, derived from sh2_SensorValue.h
typedef struct
{
    float real;       // Quaternion real component
    float i;          // Quaternion i-component
    float j;          // Quaternion j-component
    float k;          // Quaternion k-component
    float accuracy;   // Accuracy estimate [radians]
    uint8_t status;   // Reliability status of sensor (0=Unreliable ... 3=High)
    // From sh2_SensorValue.h:
    /* Status of a sensor
     *   0 - Unreliable
     *   1 - Accuracy low
     *   2 - Accuracy medium
     *   3 - Accuracy high
    */
} imu_data_t;

typedef struct
{
    double latitude;
    double longtitude;
    bool has_fix;
} gps_data_t;
//--------------------------------------------------------------------------------------------------

// Drone's state as calculated by the fusion task.
typedef struct {
    float roll;
    float pitch;
    float yaw;

    float altitude;
    float velocity;

    double latitude;
    double longtitude;

    float temperature;
} fusion_state_t;

typedef enum {
    SENSOR_BARO,
    SENSOR_IMU,
    SENSOR_GPS
} sensor_type_t;

// It's somewhat important to keep the memory footprint of this union low, because it goes into the sensorfusion queue.
// This union takes about 20 bytes of memory at the minimum because there are 4 floats in imu_data_t (r, i, j, k, accuracy)
typedef union {
    imu_data_t imu;
    baro_data_t baro;   // Joko kirjoitetaan ne omat datatyypit näille tai sitten käytetään valmiita headereista
    gps_data_t gps;
} sensor_payload_t;

// This is the message sent to fusion task
typedef struct {
    sensor_type_t type;
    int timestamp;
    sensor_payload_t data;
} sensor_msg_t;

// message sent to telemetry task
typedef struct {
    fusion_state_t state;
    int timestamp;
} fusion_msg_t;


#endif