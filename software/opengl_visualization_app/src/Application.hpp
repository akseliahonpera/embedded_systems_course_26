#pragma once

#include "Camera.hpp"
#include "Mesh.hpp"

struct Application
{
    Application(int window_width, int window_height);
    ~Application();

    void render();
    void update(float deltaTime);
    void windowResized(int new_width, int new_height);
private:
    void updateGui();
    Mesh monkey;
    Camera cam;
};
