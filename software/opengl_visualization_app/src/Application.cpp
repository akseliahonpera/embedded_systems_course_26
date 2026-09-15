#include <glm/gtc/constants.hpp>
#include <iostream>
#include "Application.hpp"
#include "imgui/imgui.h"
#include "MeshRenderer.hpp"
#include "Utils.hpp"
#include <unordered_map>


constexpr float client_timeout_time = 5.0f;


glm::vec3 convertAngles(glm::vec3 angles)
{
    float roll  = glm::radians(angles[0]);
    float pitch = glm::radians(angles[1]);
    float yaw   = glm::radians(angles[2]);

    glm::quat q = glm::angleAxis(yaw,   glm::vec3(0.0f, 1.0f, 0.0f))
                * glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f))
                * glm::angleAxis(roll,  glm::vec3(0.0f, 0.0f, 1.0f));

    return glm::eulerAngles(q);
}


DroneModel::DroneModel()
{
    transform = std::make_shared<Transform>();

    body = Mesh::fromObj(ObjLoader::loadObj("assets/drone.obj"));

    body.transform->scale = glm::vec3(0.4f);
    body.transform->setParent(transform);
    body.transform->rotation.y += glm::half_pi<float>();

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

void ClientState::updateData(ClientPtr client)
{
    if (client->client_addr != client_addr) {
        client_addr = client->client_addr;
    }

    while (client->dataAvailable()) {

        Datagram dg;
        if (client->popDatagram(dg)) {

            rotation = glm::vec3(dg.rotation[0], dg.rotation[1], dg.rotation[2]);
            coordinates = glm::vec2(dg.location[0], dg.location[1]);
            pressure = dg.pressure;
            temperature = dg.temperature;

        }
    }

    packet_loss = client->packetLoss();
}

void DroneModel::render(const Camera &cam)
{
    auto &mesh_renderer = MeshRenderer::getInstance();

    mesh_renderer.render(cam, body);

    for (auto &prop : propellers) {
        mesh_renderer.render(cam, prop);
    }
}

void DroneModel::update(float deltaTime, ClientState *show_client)
{
    if (show_client) {
        transform->rotation = convertAngles(show_client->rotation);
    }

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
    glm::vec3 rotation(0.0f);
    glm::vec2 coordinates(0.0f);
    float pressure = 0;
    float temperature = 0;

    ImGui::NewFrame();

    ImGui::Begin("Clients");

    for (auto &[addr, client_state] : active_clients) {
        ImGui::Text("%s Packet loss %.1f%%", addr.c_str(), client_state.packet_loss*100.0f);

        if (addr == show_client_addr) {
            rotation = client_state.rotation;
            coordinates = client_state.coordinates;
            pressure = client_state.pressure;
            temperature = client_state.temperature;
        } else {
            ImGui::SameLine();
            ImGui::PushID(std::hash<std::string>{}(addr));
            if (ImGui::Button("Track")) {
                show_client_addr = addr;
            }
            ImGui::PopID();
        }
    }

    ImGui::End();


    ImGui::Begin("Rotation");
    ImGui::Text("Roll %.2f, Pitch %.2f, Yaw %.2f", rotation[0], rotation[1], rotation[2]);
    if (ImGui::Button("Calibrate")) {
        std::cout << "blablaa" << std::endl;
    }

    ImGui::End();

    ImGui::Begin("Location");
    ImGui::Text("Latitude %.4f, Longitude %.4f", coordinates.x, coordinates.y);

    ImGui::End();

    ImGui::Begin("Pressure");

    ImGui::Text("%.2f hPa", pressure);
    ImGui::Text("Estimated height %.1f m", temperature);
    ImGui::SameLine();
    if (ImGui::Button("Reset zero")) {
        std::cout << "blabalaa" << std::endl;
    }

    ImGui::End();
}

void Application::update(float deltaTime)
{
    float curr_time = Utils::getTimeStamp();
    for (auto &clt : server.getClients()) {
        if (curr_time - clt->lastReceiveServerTime() > client_timeout_time) {
            if (active_clients.find(clt->client_addr) != active_clients.end()) {
                active_clients.erase(clt->client_addr);
            }
            continue;
        }
        if (active_clients.find(clt->client_addr) == active_clients.end()) {
            active_clients[clt->client_addr] = ClientState();
        }
        active_clients[clt->client_addr].updateData(clt);
    }

    updateGui();

    ClientState *show_client = nullptr;
    if (active_clients.find(show_client_addr) != active_clients.end()) {
        show_client = &active_clients[show_client_addr];
    }

    drone.update(deltaTime, show_client);
}

void Application::windowResized(int new_width, int new_height)
{
    cam.setViewPort(new_width, new_height);
}
