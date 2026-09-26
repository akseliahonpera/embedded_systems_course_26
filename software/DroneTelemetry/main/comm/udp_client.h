#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    int sock;
    struct sockaddr_in dest;
    uint32_t errors;
} udp_client_t;

esp_err_t udp_client_open(udp_client_t *client, const char *dest_ip, uint16_t port);
void udp_client_close(udp_client_t *client);

esp_err_t udp_client_send(udp_client_t *client, const void *data, size_t len);
int udp_client_receive(udp_client_t *client, void *data, size_t len);

#ifdef __cplusplus
}
#endif
