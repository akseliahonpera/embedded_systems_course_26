#include "telem_client.h"
#include "telem_packet.h"
#include "udp_client.h"

void telem_client_init(telem_client_t *client, udp_client_t *udp_client)
{
    client->udp_client = udp_client;
    client->ctrl_pkt_seq = -1;
    client->ctrl_pkt_seq_mask = 0;
}

void telem_client_send(telem_client_t *client, telem_packet_t *packet)
{
    udp_client_send(client->udp_client, packet, sizeof(telem_packet_t));
}

int telem_client_poll_command(telem_client_t *client)
{
    control_packet_t ctrl_pkt = { 0 };

    int rx_len = udp_client_receive(client->udp_client, &ctrl_pkt, sizeof(ctrl_pkt));
    if (rx_len != sizeof(ctrl_pkt)) {
        return 0;
    }

    int seq_delta = client->ctrl_pkt_seq - ctrl_pkt.packet_num;
    int ret_cmd = ctrl_pkt.command;
    if (seq_delta < 0) {
        //This sequence number is higher than any previously received
        //Shift sequence number mask and mark received
        client->ctrl_pkt_seq_mask = (client->ctrl_pkt_seq_mask << (-seq_delta)) | 0x1;
        client->ctrl_pkt_seq = ctrl_pkt.packet_num;
    } else if (seq_delta >= 64) {
        //Packet sequence number is outside of the dedublication window
        //Drop this command, since there is no quarantee of dedublication
        ret_cmd = 0;
    } else {
        if ((client->ctrl_pkt_seq_mask >> seq_delta) & 0x1) {
            //Already received command with same sequence number
            //Drop this command
            ret_cmd = 0;
        }
        //Mark sequence number as received
        client->ctrl_pkt_seq_mask |= (1 << seq_delta);
    }

    //Send ACT so that server stop resending
    //Maybe we should add another ACT_BUT_IGNORED flag when 
    //command is dropped due to other reasons (outside dedublication window)
    ctrl_pkt.command = CMD_ACT;
    udp_client_send(client->udp_client, &ctrl_pkt, sizeof(ctrl_pkt));

    return ret_cmd;
}