#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_check.h"
#include "udp_client.h"

static const char *TAG = "udp_client";

esp_err_t udp_client_open(udp_client_t *client, const char *dest_ip, uint16_t port)
{
    ESP_RETURN_ON_FALSE(dest_ip, ESP_ERR_INVALID_ARG, TAG, "null ip");

    //Convert destination address
    memset(&client->dest, 0, sizeof(client->dest));
    client->dest.sin_family = AF_INET;
    client->dest.sin_port   = htons(port);
    if (inet_pton(AF_INET, dest_ip, &client->dest.sin_addr) != 1) {
        ESP_LOGE(TAG, "Bad address '%s'", dest_ip);
        return ESP_ERR_INVALID_ARG;
    }

    //Create socket
    client->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (client->sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return ESP_FAIL;
    }

    //Disable blocking when receiving
    int flags = fcntl(client->sock, F_GETFL);
    if (fcntl(client->sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        ESP_LOGI(TAG, "Unable to set socket non blocking");
    }

    //Allow broadcasting. Not sure is useful in our application
    int on = 1;
    setsockopt(client->sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

    //Set very short timeout interval for sending
    //If there is poor wifi connection, sendto might hang because of wifi stack slowing down
    //Since we send telemetry where packet drop is expected, it is better to let packets drop
    //than wait that wifi send queue
    struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 };
    setsockopt(client->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    client->errors = 0;
    ESP_LOGI(TAG, "UDP Client created for host %s:%u", dest_ip, port);
    return ESP_OK;
}

void udp_client_close(udp_client_t *client)
{
    if (client->sock >= 0) {
        shutdown(client->sock, 0);
        close(client->sock);
        client->sock = -1;
    }
}

esp_err_t udp_client_send(udp_client_t *client, const void *data, size_t len)
{
    if (client->sock < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int tx_len = sendto(client->sock, data, len, 0,
                        (struct sockaddr *)&client->dest, sizeof(client->dest));
    if (tx_len < 0) {
        client->errors += 1;
        return ESP_FAIL;
    }
    return ESP_OK;
}


int udp_client_receive(udp_client_t *client, void *data, size_t len)
{
    struct sockaddr_storage source_addr;
    struct sockaddr_in *sin = (struct sockaddr_in*)&source_addr;
    socklen_t socklen = sizeof(source_addr);

    int rx_len = recvfrom(client->sock, data, len, 0, (struct sockaddr *)&source_addr, &socklen);

    //recvfrom receives all packets which was sent to our client port
    
    //drop packets if not ipv4
    if (source_addr.ss_family != AF_INET || socklen < sizeof(struct sockaddr_in)) {
        return 0;
    }
    
    //drop packets if sender address and port doesnt match host address and port
    if (sin->sin_port        != client->dest.sin_port || 
        sin->sin_addr.s_addr != client->dest.sin_addr.s_addr) {
        return 0;
    }

    return rx_len;
}