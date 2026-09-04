#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include "freertos/semphr.h"
#include <FreeRTOSConfig.h>

#include <nvs.h>
#include <esp_log.h>


struct Orientation{
    int x;
    int y;
    int z;
    int ax;
    int ay;
    int az;
    int latitude;
    int longitude;
    int time;

};
struct Cursor{
    int cursorFirst;
    int cursorLast;//included
};

struct communicationBuffer{
    char tx_buffer[128];
    char rx_buffer[128];
};

struct Common_pointers{
    struct Orientation* orientation_ptr;
    struct Cursor* cursor_ptr;
    struct communicationBuffer* comm_ptr;
    SemaphoreHandle_t* semaphore_ptr;
};




void createMockData(struct Orientation* ptr);
void initMockData(struct Orientation* ptr);
void initArrayOfStructs(struct Orientation* buffer);
void initCursor(struct Cursor*ptr);

void init_comm_buffer(struct communicationBuffer* ptr);

void printArrayOfStructs(struct Orientation*buffer);