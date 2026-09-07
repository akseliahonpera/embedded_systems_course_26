#include "Server.hpp"
#include "Utils.hpp"
#include <cstring>
#include <iostream>
#include <bit>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

constexpr int DEDUBLICATION_WINDOW = 64;

float Client::packetLoss() {
    return 1.0f - (static_cast<float>(std::popcount<uint64_t>(received_mask)) /
                   std::min(64, last_received_packet_num - first_received_packet_num + 1));
}

void Client::reset()
{
    buffer = DatagramBuffer();
    last_receive_server_time = 0;
    last_popped_packet_num = -1;
    last_received_packet_num = -1;
    received_mask = 0;
    first_received_packet_num = -1;
}

void Client::pushDatagram(const Datagram &dg) {
    float server_time = Utils::getTimeStamp();

    std::lock_guard guard(buffer_lock);

    if (server_time - last_receive_server_time > 5.0f) {
        reset();
    }
    if (dg.packet_num > last_popped_packet_num &&
        dg.packet_num + DEDUBLICATION_WINDOW > last_received_packet_num) {

        if (dg.packet_num > last_received_packet_num) {
            received_mask <<= dg.packet_num - last_received_packet_num;
            last_received_packet_num = dg.packet_num;
        }
        uint64_t seq_mask = 1ull << (last_received_packet_num - dg.packet_num);
        if ((received_mask & seq_mask) != 0) {
            std::cout << "Dublicate packet dropped!" << std::endl;
            return;
        }
        received_mask |= seq_mask;

        last_receive_server_time = server_time;

        if (first_received_packet_num < 0) {
            first_received_packet_num = dg.packet_num;
        }

        buffer.push(dg);

    }
}

bool Client::popDatagram(Datagram &dg) {
    std::lock_guard guard(buffer_lock);
    if (buffer.empty()) {
        return false;
    }

    dg = buffer.top();
    buffer.pop();

    last_popped_packet_num = dg.packet_num;

    return true;
}

size_t Client::dataAvailable()
{
    std::lock_guard guard(buffer_lock);
    return buffer.size();
}

Server::Server(int port)
{
    port_to_listen = port;
    running = true;
    server_thread = std::thread(&Server::serverThread, this);
}

Server::~Server()
{
    running = false;
    server_thread.join();
}

std::string sockaddr_to_string(const sockaddr_in& addr) {
    char ip_buffer[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &(addr.sin_addr), ip_buffer, INET_ADDRSTRLEN) == nullptr) {
        return "";
    }
    uint16_t port = ntohs(addr.sin_port);

    return std::string(ip_buffer) + ":" + std::to_string(port);
}


void Server::serverThread()
{
    struct sockaddr_in servaddr, cliaddr;
    int sockfd;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cout << "Socket creation failed!" << std::endl;
        return;
    }

    std::memset(&servaddr, 0, sizeof(servaddr));
    std::memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port_to_listen);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cout << "Cannot bind socket to " << port_to_listen << std::endl;
        return;
    }


    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    socklen_t len = sizeof(cliaddr);

    while (running) {
        Datagram dg;

        int received = recvfrom(sockfd, (char*)&dg, sizeof(dg), MSG_WAITALL, (struct sockaddr*)&cliaddr, &len);

        if (received != sizeof(Datagram)) {
            std::cout << "No data received!" << std::endl;
            continue;
        }

        std::string client_addr = sockaddr_to_string(cliaddr);

        std::cout << "Received data from " << client_addr << std::endl;

        if (known_clients.find(client_addr) == known_clients.end()) {
            std::lock_guard lock(clients_mutex);
            known_clients[client_addr] = std::make_shared<Client>(client_addr);
        }
        known_clients[client_addr]->pushDatagram(dg);
    }
    close(sockfd);
}

std::vector<ClientPtr> Server::getClients()
{
    std::lock_guard lock(clients_mutex);
    std::vector<ClientPtr> clients;
    for (auto& [key, value] : known_clients) {
        clients.push_back(value);
    }
    return clients;
}
