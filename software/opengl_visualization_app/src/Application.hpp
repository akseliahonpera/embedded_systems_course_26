#pragma once

#include "Camera.hpp"
#include "Mesh.hpp"
#include <array>
#include "Server.hpp"

struct DroneModel
{
    DroneModel();

    void render(const Camera &cam);
    void update(float deltaTime);

    Mesh body;
    std::array<Mesh, 4> propellers;
};


struct ClientState
{
    ClientState() {
        coordinates = glm::vec2(0);
        rotation = glm::vec3(0);
        pressure = 0.0f;
        temperature = 0.0f;
    }

    glm::vec2 coordinates;
    glm::vec3 rotation;
    float pressure;
    float temperature;
    float packet_loss;

    std::string client_addr;

    void updateData(ClientPtr client);
};

struct Application
{
    Application(int window_width, int window_height);
    ~Application();

    void render();
    void update(float deltaTime);
    void windowResized(int new_width, int new_height);
private:
    void updateGui();
    DroneModel drone;
    Camera cam;
    Server server;

    std::string show_client_addr;
    std::map<std::string, ClientState> active_clients;
};
