#pragma once

#include "telem_packet.h"
#include "udp_client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    udp_client_t *udp_client;

    int32_t telem_pkt_seq;
    int ctrl_pkt_seq;
    uint64_t ctrl_pkt_seq_mask;

    uint64_t last_valid_cmd_time;
} telem_client_t;


void telem_client_init(telem_client_t *client, udp_client_t *udp_client);

void telem_client_send(telem_client_t *client, telem_packet_t *packet);
int telem_client_poll_command(telem_client_t *client);
