#include "drone_state.h"

void initCursor(struct Cursor*ptr){
    ptr->cursorFirst = 0;
    ptr->cursorLast = 0;
}

void initMockData(struct Orientation* ptr){
    ptr->x = 0;
    ptr->y = 0;
    ptr->z = 0;
    ptr->ax = 0;
    ptr->ay = 0;
    ptr->az = 0;
    ptr->latitude = 0;
    ptr->longitude = 0;
    ptr->time = 0;
}

void initArrayOfStructs(struct Orientation* buffer){
   int buffer_size = 32;
   
   for(int i = 0; i<buffer_size;i++){
        initMockData((buffer+i));
   }
}

void printArrayOfStructs(struct Orientation*buffer){
 int buff_size = 32;
 printf("\nPrinting array of structs:");
for(int i=0;i<buff_size;i++){
    printf("\n%d , %d, %d, %d, %d, %d, %d, %d, %d",
        (buffer+i)->x,(buffer+i)->y,(buffer+i)->z,
        (buffer+i)->ax,(buffer+i)->ay,(buffer+i)->az,
        (buffer+i)->latitude,(buffer+i)->longitude,
        (buffer+i)->time);
}
}

void createMockData(struct Orientation* ptr){
    ptr->x = rand()% 10;
    ptr->y = rand()% 10;
    ptr->z = rand()% 10;
    ptr->ax = rand()% 10;
    ptr->ay = rand()% 10;
    ptr->az = rand()% 10;
    ptr->latitude = rand()% 90;
    ptr->longitude = rand()% 90;
    ptr->time = ptr->time+1;
}

void init_comm_buffer(struct communicationBuffer* ptr){
    printf("initializing comm buffer");
    for(int i = 0; i<128;i++){
        ptr->tx_buffer[i] = 0;
        ptr->rx_buffer[i] = 0;
    }
    printf("comm buffer initialized");
}

