#include <glm/gtc/constants.hpp>
#include <iostream>
#include "Application.hpp"
#include "Server.hpp"
#include "TelemPacket.hpp"
#include "imgui/imgui.h"
#include "MeshRenderer.hpp"
#include "Utils.hpp"

constexpr float client_timeout_time = 5.0f;


glm::vec3 convertAngles(glm::vec3 angles)
{
    //Our OpenGL rendering engine uses different coordinates system and rotation order
    //Angles need to be converted to that system

    float roll  = glm::radians(angles[0]);
    float pitch = glm::radians(angles[1]);
    float yaw   = glm::radians(angles[2]);

    glm::mat4 rot_mat(1.0f);

    rot_mat = glm::rotate(rot_mat, yaw,   glm::vec3(0, 1, 0));
    rot_mat = glm::rotate(rot_mat, pitch, glm::vec3(1, 0, 0));
    rot_mat = glm::rotate(rot_mat, roll,  glm::vec3(0, 0, 1));

    float pitch2, yaw2, roll2;
    glm::extractEulerAngleZYX(rot_mat, roll2, yaw2, pitch2);

    return glm::vec3(pitch2, yaw2, roll2);
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
            /*static int drop_counter = 0;
            drop_counter++;
            if ((drop_counter % 10) != 0) {
                continue;
            }*/

            last_frame = DataFrame::fromDatagram(dg);
            data_sampler.pushFrame(Utils::getTimeStamp(), last_frame);
        }
    }

    packet_loss = client->packetLoss();
    latency = client->getRoundTripLatency();
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
        //transform->rotation = convertAngles(show_client->data_sampler.getSample(Utils::getTimeStamp() - 0.25f).rotation);
        transform->rotation = convertAngles(show_client->last_frame.rotation);
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
    glm::vec3 location(0.0f);
    float pressure = 0;
    float temperature = 0;

    ImGui::NewFrame();

    ImGui::Begin("Clients");
        for (auto &[addr, client_state] : active_clients) {
            ImGui::Text("%s Packet loss %.1f%%  Ping %.1f ms", addr.c_str(), client_state.packet_loss*100.0f, client_state.latency*1000.0f);

            if (addr == show_client_addr) {
                rotation = client_state.last_frame.rotation;
                coordinates = client_state.last_frame.coordinates;
                location = client_state.last_frame.location;
                pressure = client_state.last_frame.pressure;
                temperature = client_state.last_frame.temperature;
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

    ImGui::Begin("IMU");
        ImGui::Text("Roll %.2f, Pitch %.2f, Yaw %.2f", rotation[0], rotation[1], rotation[2]);

        if (ImGui::BeginMenu("Commands")) {
            if (ImGui::Button("Disable IMU"))
                sendCommandToActiveClient(CMD_DISABLE_IMU);

            if (ImGui::Button("Enable IMU"))
                sendCommandToActiveClient(CMD_ENABLE_IMU);

            if (ImGui::Button("Reset IMU"))
                sendCommandToActiveClient(CMD_RESET_IMU);

            if (ImGui::Button("Measure covariance"))
                sendCommandToActiveClient(CMD_IMU_MEASURE_COV);

            if (ImGui::Button("Measure bias"))
                sendCommandToActiveClient(CMD_IMU_MEASURE_BIAS);

            ImGui::EndMenu();
        }

    ImGui::End();

    ImGui::Begin("Location");
        ImGui::Text("Latitude %.4f, Longitude %.4f", coordinates.x, coordinates.y);
        ImGui::Text("Relative location (X,Y,Z): %.4f, %.4f, %.4f", location.x, location.y, location.z);

        if (ImGui::BeginMenu("Commands")) {
            if (ImGui::Button("Start fusion"))
                sendCommandToActiveClient(CMD_FUSION_START);

            if (ImGui::Button("Stop fusion"))
                sendCommandToActiveClient(CMD_FUSION_STOP);

            if (ImGui::Button("Enable GNSS"))
                sendCommandToActiveClient(CMD_ENABLE_GPS);

            if (ImGui::Button("Disable GNSS"))
                sendCommandToActiveClient(CMD_DISABLE_GPS);

            if (ImGui::Button("Reset GNSS"))
                sendCommandToActiveClient(CMD_RESET_GPS);

            ImGui::EndMenu();
        }

    ImGui::End();

    ImGui::Begin("Pressure / Temperature");
        ImGui::Text("Pressure %.2f hPa", pressure);
        ImGui::Text("Temperature %.2f °C", temperature);


        if (ImGui::BeginMenu("Commands")) {
            if (ImGui::Button("Enable barometer"))
                sendCommandToActiveClient(CMD_ENABLE_BAROMETER);

            if (ImGui::Button("Disable barometer"))
                sendCommandToActiveClient(CMD_DISABLE_BAROMETER);

            if (ImGui::Button("Reset barometer"))
                sendCommandToActiveClient(CMD_RESET_BAROMETER);

            ImGui::EndMenu();
        }

    ImGui::End();


    ImGui::Begin("Logging");
    ImGui::InputText("Output file", ui_data.logging_file, sizeof(ui_data.logging_file));
        ImGui::Checkbox("Rotation", &ui_data.log_rotation);
        ImGui::Checkbox("Relative location", &ui_data.log_relative_location);
        ImGui::Checkbox("Coordinates", &ui_data.log_coordinates);
        ImGui::Checkbox("Pressure", &ui_data.log_pressure);
        ImGui::Checkbox("Temperature", &ui_data.log_temperature);
        ImGui::SliderFloat("Samplerate", &ui_data.log_sample_rate, 1, 100);
        if (ImGui::Button("Apply")) {
            logger.setOutputFile(std::string(ui_data.logging_file));
            logger.enableCoordinatesLogging(ui_data.log_coordinates);
            logger.enableRotationLogging(ui_data.log_rotation);
            logger.enableRelativeLocationLogging(ui_data.log_relative_location);
            logger.enablePressureLogging(ui_data.log_pressure);
            logger.enableTemperatureLogging(ui_data.log_temperature);
            logger.setSampleRate(ui_data.log_sample_rate);
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

    if (show_client) {
        logger.log(show_client->data_sampler);
    }
}

void Application::sendCommandToActiveClient(ControlCommand cmd)
{
    auto client = server.getClient(show_client_addr);
    if (client != nullptr) {
        client->sendControlCommand(cmd);
    }
}

void Application::windowResized(int new_width, int new_height)
{
    cam.setViewPort(new_width, new_height);
}
