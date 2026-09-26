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
#include <cstdlib>

constexpr int DEDUBLICATION_WINDOW = 64;
constexpr float CTRL_RESEND_TIMER = 1.0f;

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

void Client::sendControlCommand(ControlCommand cmd)
{
    std::lock_guard guard(ctrl_cmd_mutex);

    ControlDatagram dg;
    dg.command = cmd;
    dg.packed_num = ctrl_cmd_sequence_num++;

    ctrl_cmd_buffer.push_back({-1.0f, dg});

    if (cmd == CMD_PING) {
        last_ping_time = Utils::getTimeStamp();
    }
}


void Client::controlCommandAct(int packet_num)
{
    std::lock_guard guard(ctrl_cmd_mutex);

    for (size_t i = 0; i < ctrl_cmd_buffer.size(); i++) {
        if (ctrl_cmd_buffer[i].second.packed_num == packet_num) {
            roundtrip_latency = Utils::getTimeStamp() - ctrl_cmd_buffer[i].first + CTRL_RESEND_TIMER;

            std::swap(ctrl_cmd_buffer[i], ctrl_cmd_buffer.back());
            ctrl_cmd_buffer.pop_back();
            return;
        }
    }
}

int Client::controlCommandsWaiting()
{
    std::lock_guard guard(ctrl_cmd_mutex);
    return ctrl_cmd_buffer.size();
}

std::vector<ControlDatagram> Client::queuedControlCommands(bool reset_timer)
{
    float curr_time = Utils::getTimeStamp();

    std::lock_guard guard(ctrl_cmd_mutex);

    std::vector<ControlDatagram> dgs;

    for (auto &[resend_time, ctrl_dg] : ctrl_cmd_buffer) {
        if (curr_time > resend_time) {
            if (reset_timer)
                resend_time = curr_time + CTRL_RESEND_TIMER;

            dgs.push_back(ctrl_dg);
        }
    }
    return dgs;
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

sockaddr_in string_to_sockaddr(std::string addr_str) {
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(sockaddr_in));

    auto delim = addr_str.find(":");
    std::string host = addr_str.substr(0, delim);
    int port = std::atoi(addr_str.substr(delim+1, addr_str.length()-delim-1).c_str());

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
    return addr;
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
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    socklen_t len = sizeof(cliaddr);

    while (running) {
        float curr_time = Utils::getTimeStamp();
        for (auto [client_addr, client] : known_clients) {

            if (!client->controlCommandsWaiting() &&
                curr_time - client->getLastPingTime() > 1.0f) {
                client->sendControlCommand(CMD_PING);
            }

            for (auto cmd : client->queuedControlCommands(true)) {
                std::cout << "Sending CTRL packet with seq " << cmd.packed_num << " to " << client_addr << std::endl;

                sockaddr_in saddr = string_to_sockaddr(client_addr);
                sendto(sockfd, &cmd, sizeof(cmd), 0, (sockaddr*)&saddr, sizeof(sockaddr_in));
            }
        }

        union {
            Datagram dg;
            ControlDatagram ctrl_dg;
        };

        int received = recvfrom(sockfd, (char*)&dg, sizeof(dg), MSG_WAITALL, (struct sockaddr*)&cliaddr, &len);

        std::string client_addr = sockaddr_to_string(cliaddr);

        if (received == sizeof(ControlDatagram) && ctrl_dg.command == CMD_ACT) {
            if (known_clients.find(client_addr) == known_clients.end()) {
                std::cout << "Received act from unknown client!" << std::endl;
                return;
            }

            std::cout << "Received ACT with seq " << ctrl_dg.packed_num << " from " << client_addr << std::endl;

            known_clients[client_addr]->controlCommandAct(ctrl_dg.packed_num);

            continue;
        }

        if (received != sizeof(Datagram)) {
            std::cout << "No data received!" << std::endl;
            continue;
        }


        //std::cout << "Received data from " << client_addr << std::endl;

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

ClientPtr Server::getClient(std::string addr)
{
    std::lock_guard lock(clients_mutex);
    if (known_clients.find(addr) == known_clients.end()) {
        return nullptr;
    }
    return known_clients[addr];
}
