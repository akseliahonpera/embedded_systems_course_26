#pragma once

#include "includes.hpp"

#include "TelemPacket.hpp"


inline glm::quat taitBryatToQuat(glm::vec3 angles)
{
    float roll  = glm::radians(angles[0]);
    float pitch = glm::radians(angles[1]);
    float yaw   = glm::radians(angles[2]);

    glm::mat4 matYXZ = glm::eulerAngleYXZ(yaw, pitch, roll);

    return glm::quat(matYXZ);
}

inline glm::vec3 quatToTaitBryat(glm::quat q)
{
    glm::mat4 rot_mat = glm::mat4_cast(q);

    float pitch, yaw, roll;
    glm::extractEulerAngleYXZ(rot_mat, yaw, pitch, roll);
    return glm::degrees(glm::vec3(roll, pitch, yaw));
}



struct DataFrame
{
    DataFrame() {
        coordinates = glm::vec2(0, 0);
        rotation = glm::vec3(0, 0, 0);

        pressure = 0;
        temperature = 0;
    }


    glm::vec2 coordinates;
    glm::vec3 rotation;
    float pressure;
    float temperature;

    static DataFrame fromDatagram(const Datagram &dg) {
        DataFrame frame;

        frame.rotation = glm::vec3(dg.rotation[0], dg.rotation[1], dg.rotation[2]);
        frame.coordinates = glm::vec2(dg.location[0], dg.location[1]);
        frame.pressure = dg.pressure;
        frame.temperature = dg.temperature;

        return frame;
    }

    static DataFrame interpolate(float t, DataFrame &a, DataFrame &b) {
        DataFrame intrp;

        intrp.coordinates = b.coordinates * t + a.coordinates * (1.0f - t);
        intrp.pressure    = b.pressure    * t + a.pressure    * (1.0f - t);
        intrp.temperature = b.temperature * t + a.temperature * (1.0f - t);

        intrp.rotation = quatToTaitBryat(glm::slerp(taitBryatToQuat(a.rotation), taitBryatToQuat(b.rotation), t));

        return intrp;
    };

    void print();
};
