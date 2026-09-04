
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "drone_state.h"
//check if compiler freaks out about this


//set here the ip of server and also port, keep lwip_ipv4 as 1
#define HOST_IP_ADDR "192.168.101.114"
#define LWIP_IPV4 1
#define PORT 7777