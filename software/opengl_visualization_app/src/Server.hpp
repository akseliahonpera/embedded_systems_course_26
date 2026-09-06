#pragma once

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
    float estimated_height;

    int packet_num;
    float time;
};

struct Client
{
    Client(std::string addr) {
        client_addr = addr;
    }

    std::string client_addr;

    void queueDatagram(const Datagram &dg);
    bool dequeueDatagram(Datagram &dg);
    bool dataAvailable();
private:
    std::queue<Datagram> buffer;
    std::mutex buffer_lock;
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
