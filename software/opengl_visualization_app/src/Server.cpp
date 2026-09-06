#include "Server.hpp"
#include <cstring>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>


void Client::queueDatagram(const Datagram &dg) {
    std::lock_guard guard(buffer_lock);
    buffer.push(dg);
}

bool Client::dequeueDatagram(Datagram &dg) {
    std::lock_guard guard(buffer_lock);
    if (buffer.empty()) {
        return false;
    }

    dg = buffer.front();
    buffer.pop();

    return true;
}

bool Client::dataAvailable()
{
    std::lock_guard guard(buffer_lock);
    return !buffer.empty();
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

        if (known_clients.find(client_addr) != known_clients.end()) {
            std::lock_guard lock(clients_mutex);
            known_clients[client_addr] = std::make_shared<Client>(client_addr);
        }

        known_clients[client_addr]->queueDatagram(dg);
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
