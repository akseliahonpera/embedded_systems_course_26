#pragma once

#include "Camera.hpp"
#include "Mesh.hpp"
#include <array>
#include <cstring>
#include "Server.hpp"
#include "DataSampler.hpp"
#include "DataLogger.hpp"

struct ClientState;

struct DroneModel
{
    DroneModel();

    void render(const Camera &cam);
    void update(float deltaTime, ClientState *show_client);

    std::shared_ptr<Transform> transform;

    Mesh body;
    std::array<Mesh, 4> propellers;
};


struct ClientState
{
    DataSampler data_sampler;
    DataFrame last_frame;

    float packet_loss;
    float latency;

    std::string client_addr;

    void updateData(ClientPtr client);
};

struct UiStaticData
{
    UiStaticData()
    {
        memset(this, 0, sizeof(UiStaticData));
        strcpy(logging_file, "data.csv");
        log_sample_rate = 5;
    }

    char logging_file[128];
    bool log_rotation;
    bool log_coordinates;
    bool log_relative_location;
    bool log_temperature;
    bool log_pressure;
    float log_sample_rate;
};


struct Application
{
    Application(int window_width, int window_height);
    ~Application();

    void render();
    void update(float deltaTime);
    void windowResized(int new_width, int new_height);
private:
    void sendCommandToActiveClient(ControlCommand cmd);
    void updateGui();
    DroneModel drone;
    Camera cam;
    Server server;
    Logging::DataLogger logger;

    UiStaticData ui_data;

    std::string show_client_addr;
    std::map<std::string, ClientState> active_clients;
};
