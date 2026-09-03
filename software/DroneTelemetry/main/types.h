#ifndef TYPES_H
#define TYPES_H

#include "bmp5_defs.h" 
#include "sh2_SensorValue.h"
#include <stdbool.h>
#include <stdint.h> 

//En tiiä tartteeko näitä, voi käyttää periaatteessa valmiitakin tietorakenteita muista headereista
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
    bool paska;
    /* TODO */
    /* lat, longt, has_fix ?*/
} gps_data_t;
//--------------------------------------------------------------------------------------------------


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

typedef struct {
    sensor_type_t type;
    int timestamp;
    sensor_payload_t data;
} sensor_message_t;



#endif