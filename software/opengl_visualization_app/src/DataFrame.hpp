#pragma once

#include "includes.hpp"

#include "TelemPacket.hpp"
#include <cmath>


inline glm::vec3 quatToTaitBryat(glm::quat q)
{
    // Same equations and output ordering as firmware q_to_ypr() in imu/euler.c.
    const float r = q.w, i = q.x, j = q.y, k = q.z;
    const float yaw_num = 2.0f * i * j + 2.0f * r * k;
    const float yaw_den = 2.0f * r * r + 2.0f * i * i - 1.0f;
    const float yaw = std::atan2(yaw_num, yaw_den);
    float pitch_arg = 2.0f * j * k + 2.0f * r * i;
    pitch_arg = glm::clamp(pitch_arg, -1.0f, 1.0f);
    const float pitch = std::asin(pitch_arg);
    const float roll_num = -2.0f * i * k + 2.0f * r * j;
    const float roll_den = 2.0f * r * r + 2.0f * k * k - 1.0f;
    const float roll = std::atan2(roll_num, roll_den);
    return glm::degrees(glm::vec3(roll, pitch, yaw));
}




struct DataFrame
{
    DataFrame() {
        coordinates = glm::vec2(0, 0);
        quaternion = glm::vec4(1, 0, 0, 0);
        firmware_rotation = glm::vec3(0, 0, 0);
        rotation = glm::vec3(0, 0, 0);

        pressure = 0;
        temperature = 0;
    }


    glm::vec2 coordinates;
    glm::vec4 quaternion; // w, x, y, z
    glm::vec3 firmware_rotation; // roll, pitch, yaw from q_to_ypr convention
    glm::vec3 rotation;
    glm::vec3 location;
    float pressure;
    float temperature;

    static DataFrame fromDatagram(const Datagram &dg) {
        DataFrame frame;

        frame.quaternion = glm::vec4(dg.rotation[0], dg.rotation[1], dg.rotation[2], dg.rotation[3]);
        frame.firmware_rotation = quatToTaitBryat(glm::quat(frame.quaternion.x, frame.quaternion.y,
                                                            frame.quaternion.z, frame.quaternion.w));
        frame.rotation = frame.firmware_rotation;
        frame.rotation.x = -frame.firmware_rotation.x;
        frame.coordinates = glm::vec2(dg.coordinates[0], dg.coordinates[1]);
        frame.location = glm::vec3(dg.location[0], dg.location[1], dg.location[2]);
        frame.pressure = dg.pressure;
        frame.temperature = dg.temperature;

        return frame;
    }
    
    // static DataFrame fromDatagram(const Datagram &dg) {
    //     DataFrame frame;

    //     // TODO: akselien fixailua. vaihoin vaan akselien paikkaa, pitäis kattoa oikeat akselit ja miten päin levyä pidetään.
    //     // yllä uloskommentoituna orkkiskoodi
    //     glm::vec3 euler = quatToTaitBryat(
    //         glm::quat(dg.rotation[0], dg.rotation[1], dg.rotation[2], dg.rotation[3]));
    //     frame.rotation = glm::vec3(-euler.y, euler.x, euler.z); // pitch, roll, yaw

    //     frame.coordinates = glm::vec2(dg.coordinates[0], dg.coordinates[1]);
    //     frame.location = glm::vec3(dg.location[0], dg.location[1], dg.location[2]);
    //     frame.pressure = dg.pressure;
    //     frame.temperature = dg.temperature;

    //     return frame;
    // }

    static DataFrame interpolate(float t, DataFrame &a, DataFrame &b) {
        DataFrame intrp;

        intrp.coordinates = b.coordinates * t + a.coordinates * (1.0f - t);
        intrp.pressure    = b.pressure    * t + a.pressure    * (1.0f - t);
        intrp.temperature = b.temperature * t + a.temperature * (1.0f - t);
        intrp.location    = b.location    * t + a.location    * (1.0f - t);

        const glm::quat q = glm::slerp(
            glm::quat(a.quaternion.x, a.quaternion.y, a.quaternion.z, a.quaternion.w),
            glm::quat(b.quaternion.x, b.quaternion.y, b.quaternion.z, b.quaternion.w), t);
        intrp.quaternion = glm::vec4(q.w, q.x, q.y, q.z);
        intrp.firmware_rotation = quatToTaitBryat(q);
        intrp.rotation = intrp.firmware_rotation;
        intrp.rotation.x = -intrp.firmware_rotation.x;

        return intrp;
    };

    void print();
};
