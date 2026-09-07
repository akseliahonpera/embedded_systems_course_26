#pragma once

#include "Utils.hpp"
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <map>
#include <atomic>


struct  __attribute__((__packed__)) Datagram
{
    float location[2];
    float rotation[3];
    float pressure;
    float temperature;

    int packet_num;
    float time;
};

class CompareDatagramPacketNum
{
public:
    bool operator() (const Datagram &a, const Datagram &b)
    {
        return (a.packet_num > b.packet_num);
    }
};

typedef std::priority_queue<Datagram, std::vector<Datagram>, CompareDatagramPacketNum> DatagramBuffer;

struct Client
{
    Client(std::string addr) {
        reset();
        client_addr = addr;
    }

    std::string client_addr;

    void pushDatagram(const Datagram &dg);
    bool popDatagram(Datagram &dg);
    size_t dataAvailable();

    void reset();

    float lastReceiveServerTime() {
        return last_receive_server_time;
    }
    float packetLoss();

private:
    DatagramBuffer buffer;
    std::mutex buffer_lock;

    float last_receive_server_time;

    int last_received_packet_num;
    int last_popped_packet_num;

    int first_received_packet_num;

    uint64_t received_mask;
};

typedef std::shared_ptr<Client> ClientPtr;

struct Server
{
    Server(int port);
    ~Server();

    std::vector<ClientPtr> getClients();

private:
    int port_to_listen;

    std::atomic<bool> running;
    void serverThread();

    std::thread server_thread;

    std::map<std::string, ClientPtr> known_clients;
    std::mutex clients_mutex;
};
