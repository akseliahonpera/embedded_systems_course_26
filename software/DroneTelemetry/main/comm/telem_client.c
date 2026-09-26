#include "telem_client.h"
#include "telem_packet.h"
#include "udp_client.h"
#include "esp_timer.h"

void telem_client_init(telem_client_t *client, udp_client_t *udp_client)
{
    client->udp_client = udp_client;
    client->telem_pkt_seq = 0;
    client->ctrl_pkt_seq = -1;
    client->ctrl_pkt_seq_mask = 0;
    client->last_valid_cmd_time = 0;
}

void telem_client_send(telem_client_t *client, telem_packet_t *packet)
{
    packet->packet_num = client->telem_pkt_seq++;
    udp_client_send(client->udp_client, packet, sizeof(telem_packet_t));
}

int telem_client_poll_command(telem_client_t *client)
{
    control_packet_t ctrl_pkt = { 0 };

    int rx_len = udp_client_receive(client->udp_client, &ctrl_pkt, sizeof(ctrl_pkt));
    if (rx_len != sizeof(ctrl_pkt)) {
        return 0;
    }

    uint64_t curr_time = esp_timer_get_time()/1000;

    int seq_delta = client->ctrl_pkt_seq - ctrl_pkt.packet_num;
    int ret_cmd = ctrl_pkt.command;
    if (seq_delta < 0) {
        //This sequence number is higher than any previously received
        //Shift sequence number mask and mark received
        client->ctrl_pkt_seq_mask = (client->ctrl_pkt_seq_mask << (-seq_delta)) | 0x1;
        client->ctrl_pkt_seq = ctrl_pkt.packet_num;
    } else if (seq_delta >= 64) {
        //Packet sequence number is outside of the dedublication window
        //Drop this command if there is recently received valid commands
        if (curr_time - client->last_valid_cmd_time > 4000) {
            client->ctrl_pkt_seq = ctrl_pkt.packet_num;
            client->ctrl_pkt_seq_mask = 1;
        } else {
            ret_cmd = 0;
        }
    } else {
        if ((client->ctrl_pkt_seq_mask >> seq_delta) & 0x1) {
            //Already received command with same sequence number
            //Drop this command
            ret_cmd = 0;
        }
        //Mark sequence number as received
        client->ctrl_pkt_seq_mask |= (1 << seq_delta);
    }

    if (ret_cmd) {
        client->last_valid_cmd_time = curr_time;
    }

    //Send ACT so that server stop resending
    //Maybe we should add another ACT_BUT_IGNORED flag when 
    //command is dropped due to other reasons (outside dedublication window)
    ctrl_pkt.command = CMD_ACT;
    udp_client_send(client->udp_client, &ctrl_pkt, sizeof(ctrl_pkt));

    return ret_cmd;
}
