#include <glm/ext/scalar_constants.hpp>
#include <iostream>
#include "Application.hpp"
#include "imgui/imgui.h"
#include "MeshRenderer.hpp"
#include "Utils.hpp"

DroneModel::DroneModel()
{
    body = Mesh::fromObj(ObjLoader::loadObj("assets/drone.obj"));

    body.transform->scale = glm::vec3(0.4f);

    Mesh prop = Mesh::fromObj(ObjLoader::loadObj("assets/prop.obj"));

    glm::vec2 prop_locations[] = {
    { 3.5f,  4.1f},
    {-3.5f,  4.1f},
    { 3.5f, -4.1f},
    {-3.5f, -4.1f}
    };

    for (int i = 0; i < 4; i++) {
        propellers[i] = prop;
        propellers[i].transform->setParent(body.transform);
        propellers[i].transform->position.x = prop_locations[i].x;
        propellers[i].transform->position.z = prop_locations[i].y;
        propellers[i].transform->rotation.y += 0.3*i;
    }
}

void DroneModel::render(const Camera &cam)
{
    auto &mesh_renderer = MeshRenderer::getInstance();

    mesh_renderer.render(cam, body);

    for (auto &prop : propellers) {
        mesh_renderer.render(cam, prop);
    }
}

void DroneModel::update(float deltaTime)
{
    body.transform->rotation.x = glm::sin(Utils::getTimeStamp()*0.5f)*0.5f;
    body.transform->rotation.y = glm::cos(Utils::getTimeStamp()*1.5f)*0.75f - glm::pi<float>()/2;

    for (auto &prop : propellers) {
        prop.transform->rotation.y += 25.0f * deltaTime;
    }
}

Application::Application(int window_width, int window_height): cam(window_width, window_height), server(1234)
{
    cam.transform->position.z += 5;
    cam.transform->position.y += 3;
    cam.transform->rotation.x -= glm::pi<float>() / 6;
}

Application::~Application()
{

}

void Application::render()
{
    drone.render(cam);
}

void Application::updateGui()
{
    ImGui::NewFrame();

    ImGui::Begin("Rotation");
    ImGui::Text("Pitch %.2f, Yaw %.2f, Roll %.2f", 0.0f, 0.0f, 0.0f);
    if (ImGui::Button("Calibrate")) {
        std::cout << "blablaa" << std::endl;
    }

    ImGui::End();

    ImGui::Begin("Location");
    ImGui::Text("Latitude %.4f, Longitude %.4f", 0.0f, 0.0f);

    ImGui::End();

    ImGui::Begin("Pressure");

    ImGui::Text("%.2f hPa", 0.0f);
    ImGui::Text("Estimated height %.1f m", 0.0f);
    ImGui::SameLine();
    if (ImGui::Button("Reset zero")) {
        std::cout << "blabalaa" << std::endl;
    }

    ImGui::End();
}

void Application::update(float deltaTime)
{
    updateGui();
    drone.update(deltaTime);
}

void Application::windowResized(int new_width, int new_height)
{
    cam.setViewPort(new_width, new_height);
}
