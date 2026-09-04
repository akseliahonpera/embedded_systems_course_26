#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include "freertos/semphr.h"
#include <FreeRTOSConfig.h>

#include <nvs.h>
#include <esp_log.h>
#include "basic_freertos_smp_usage.h"
#include "sanityChecks.h"
#include "networking.h"
#include "udp_client.c"
#include "drone_state.h"

#define CORE0       0
// only define xCoreID CORE1 as 1 if this is a multiple core processor target, else define it as tskNO_AFFINITY
#define CORE1       1
#define ORIENTATION_BUFFER_SIZE 32



void init_semaphore( void * pvParameters );

void orientation_to_tx_buffer(struct Orientation*orientation_ptr, struct communicationBuffer* comm_ptr);

void orientation_to_tx_buffer(struct Orientation*orientation_ptr, struct communicationBuffer* comm_ptr){
    snprintf(comm_ptr->tx_buffer, 128,"\n%d , %d, %d, %d, %d, %d, %d, %d, %d",
        orientation_ptr->x,
        orientation_ptr->y,
        orientation_ptr->z,
        orientation_ptr->ax,
        orientation_ptr->ay,
        orientation_ptr->az,
        orientation_ptr->latitude,
        orientation_ptr->longitude,
        orientation_ptr->time
    );
}


static void msg_handler_task(void *arg){
    printf("Initializing msg_handler_task start");
    struct Common_pointers* c_ptrs= (struct Common_pointers*)arg;
    struct Orientation* orientation_ptr =  c_ptrs->orientation_ptr;
    struct communicationBuffer* comm_ptr =  c_ptrs->comm_ptr;
    SemaphoreHandle_t* xSemaphore = c_ptrs->semaphore_ptr;
    printf("Initializing msg_handler_task finished");
    while(1){
        if(*xSemaphore != NULL ){ 
            if( xSemaphoreTake( *xSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
                orientation_to_tx_buffer(orientation_ptr, comm_ptr);
                xSemaphoreGive( *xSemaphore ); 
                vTaskDelay(pdMS_TO_TICKS(500)); 
            }else{
                /* We could not obtain the semaphore and can therefore not access * 
                // the shared resource safely. * } * } * } */
                printf("\nsemaphore could not be acquired");
            }
        }
    }
    //jonkinlainen logiikka tälle vai jätetäänkö ik looppiin?
    vTaskDelete(NULL);
}

static void orientation_task(void *arg){
    struct Common_pointers* c_ptrs= (struct Common_pointers*)arg;
    struct Orientation* orientation_ptr =  c_ptrs->orientation_ptr;
    SemaphoreHandle_t* xSemaphore = c_ptrs->semaphore_ptr;
    int buffsize = ORIENTATION_BUFFER_SIZE;

    while(1){
        if(*xSemaphore != NULL ){ 
            /* See if we can obtain the semaphore. If the semaphore is not available  
             wait 10 ticks to see if it becomes free. */
            if( xSemaphoreTake( *xSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
                /* We were able to obtain the semaphore and can now access the 
                // shared resource.*/ 
                createMockData(orientation_ptr);
                /* We have finished accessing the shared resource. Release the  
                 semaphore. */ 
                xSemaphoreGive( *xSemaphore ); 
                vTaskDelay(pdMS_TO_TICKS(500)); 
            }else{
                /* We could not obtain the semaphore and can therefore not access * 
                // the shared resource safely. * } * } * } */
                printf("\nsemaphore could not be acquired");

            }
    }
    //jonkinlainen logiikka tälle vai jätetäänkö ik looppiin?
    
}
vTaskDelete(NULL);
}


void init_semaphore( void * pvParameters ){
    struct Common_pointers* c_ptrs= (struct Common_pointers*)pvParameters;
    SemaphoreHandle_t* xSemaphore = c_ptrs->semaphore_ptr;
    *xSemaphore= xSemaphoreCreateMutex();
    if( *xSemaphore != NULL )
        {
        printf("\nSemaphore created");
        }
    else{
        printf("\nSemaphore was null after initialization, something is wrong!");
        }
    }

void app_main(void)
{  
    //tietorakenteiden alustus
    static struct communicationBuffer commBuffer;
    struct communicationBuffer* comm_buff_ptr =  &commBuffer;
    static struct Cursor cursor;
    struct Cursor *cursor_ptr = &cursor;
    static struct Orientation orientation_instance;
    struct Orientation *orientation_ptr  = &orientation_instance;
    static struct Orientation orientation_buffer[32];
    struct Orientation *orientation_buffer_ptr = orientation_buffer;
    static SemaphoreHandle_t xSemaphore;
    SemaphoreHandle_t*semaphore_ptr =&xSemaphore;
    //use 1 struct to hold references for all relevant state information structs
    static struct Common_pointers common_pointers = {&orientation_instance, &cursor, &commBuffer, &xSemaphore};
    struct Common_pointers* common_pointers_ptr = &common_pointers;
    init_semaphore((void*)common_pointers_ptr);
    print("\nInitiaiting buffers");
    initArrayOfStructs(orientation_buffer_ptr);
    init_comm_buffer(comm_buff_ptr);
    initMockData(orientation_ptr);
    initCursor(cursor_ptr);
    printArrayOfStructs(orientation_buffer_ptr);
    vTaskDelay(2000);
    //initialize semaphore(s)
   

    
    start_wifi();
    print("Self wifi startup");

    
    //Task instantiation
    xTaskCreatePinnedToCore(msg_handler_task, "pinned_task0_core0",4096,(void*)common_pointers_ptr, TASK_PRIO_3, NULL, CORE0);
    xTaskCreatePinnedToCore(orientation_task, "pinned_task1_core1",4096,(void*)common_pointers_ptr, TASK_PRIO_3, NULL, CORE1);
    xTaskCreate(udp_client_task, "udp_client", 4096, (void*)common_pointers_ptr, 5, NULL);
    print("\nInitiation complete.");
}



