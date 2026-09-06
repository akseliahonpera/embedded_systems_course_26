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
};
