#pragma once

#include "Utils.hpp"
#include <stdint.h>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <map>
#include <atomic>


enum DeviceStatusFlag: uint32_t
{
    STATUS_GNSS_ENABLED = 1 << 0,
    STATUS_GNSS_HAS_FIX = 1 << 1
};


struct  __attribute__((__packed__)) Datagram
{
    float location[2];
    float rotation[3];
    float velocity[3];
    float pressure;
    float temperature;

    uint32_t status;

    int32_t packet_num;
    float time;
};

enum ControlCommand: uint32_t
{
    CMD_ACT                 = 1 << 0,
    CMD_PING                = 1 << 1,
    CMD_CALIBRATE_IMU       = 1 << 2,
    CMD_CALIBRATE_COMPASS   = 1 << 3,
    CMD_CALIBRATE_BAROMETER = 1 << 4,
    CMD_DISABLE_GPS         = 1 << 5,
    CMD_ENABLE_GPS          = 1 << 6
};


struct __attribute__((__packed__)) ControlDatagram
{
    int32_t command;
    int32_t packed_num;
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
typedef std::vector<std::pair<float, ControlDatagram>> ControlDatagramBuffer;

struct Client
{
    Client(std::string addr) {
        reset();
        client_addr = addr;
        ctrl_cmd_sequence_num = 0;
    }

    std::string client_addr;

    void pushDatagram(const Datagram &dg);
    bool popDatagram(Datagram &dg);
    size_t dataAvailable();

    void sendControlCommand(ControlCommand cmd);

    void reset();

    float lastReceiveServerTime() {
        return last_receive_server_time;
    }
    float packetLoss();

    void controlCommandAct(int packet_num);
    std::vector<ControlDatagram> queuedControlCommands(bool reset_resend_timer);

private:
    DatagramBuffer buffer;
    std::mutex buffer_lock;

    float last_receive_server_time;

    int last_received_packet_num;
    int last_popped_packet_num;

    int first_received_packet_num;

    uint64_t received_mask;

    std::mutex ctrl_cmd_mutex;
    int32_t ctrl_cmd_sequence_num;
    ControlDatagramBuffer ctrl_cmd_buffer;
};

typedef std::shared_ptr<Client> ClientPtr;

struct Server
{
    Server(int port);
    ~Server();

    std::vector<ClientPtr> getClients();
    ClientPtr getClient(std::string addr);

private:
    int port_to_listen;

    std::atomic<bool> running;
    void serverThread();

    std::thread server_thread;

    std::map<std::string, ClientPtr> known_clients;
    std::mutex clients_mutex;
};
