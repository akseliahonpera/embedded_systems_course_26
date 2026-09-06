#include <iostream>
#include "Application.hpp"
#include "imgui/imgui.h"
#include "MeshRenderer.hpp"

Application::Application(int window_width, int window_height): cam(window_width, window_height)
{
    monkey = Mesh::fromObj(ObjLoader::loadObj("assets/monkey.obj"));

    cam.transform->position.z += 5;
}

Application::~Application()
{

}

void Application::render()
{
    auto &mesh_renderer = MeshRenderer::getInstance();

    mesh_renderer.render(cam, monkey);
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

    monkey.transform->rotation.y -= 2.0f * deltaTime;
    monkey.transform->rotation.x += 1.0f * deltaTime;
}

void Application::windowResized(int new_width, int new_height)
{
    cam.setViewPort(new_width, new_height);
}
