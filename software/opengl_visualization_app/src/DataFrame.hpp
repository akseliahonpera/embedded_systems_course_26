#pragma once

#include "includes.hpp"

#include "TelemPacket.hpp"


inline glm::quat taitBryatToQuat(glm::vec3 angles)
{
    float roll  = glm::radians(angles[0]);
    float pitch = glm::radians(angles[1]);
    float yaw   = glm::radians(angles[2]);

    glm::mat4 matYXZ = glm::eulerAngleZYX(yaw, pitch, roll);

    return glm::quat(matYXZ);
}

inline glm::vec3 quatToTaitBryat(glm::quat q)
{
    glm::mat4 rot_mat = glm::mat4_cast(q);

    float pitch, yaw, roll;
    glm::extractEulerAngleZYX(rot_mat, yaw, pitch, roll);
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
    glm::vec3 location;
    float pressure;
    float temperature;

    static DataFrame fromDatagram(const Datagram &dg) {
        DataFrame frame;

        frame.rotation =  quatToTaitBryat(glm::quat(dg.rotation[0], dg.rotation[1], dg.rotation[2], dg.rotation[3]));
        frame.coordinates = glm::vec2(dg.coordinates[0], dg.coordinates[1]);
        frame.location = glm::vec3(dg.location[0], dg.location[1], dg.location[2]);
        frame.pressure = dg.pressure;
        frame.temperature = dg.temperature;

        return frame;
    }

    static DataFrame interpolate(float t, DataFrame &a, DataFrame &b) {
        DataFrame intrp;

        intrp.coordinates = b.coordinates * t + a.coordinates * (1.0f - t);
        intrp.pressure    = b.pressure    * t + a.pressure    * (1.0f - t);
        intrp.temperature = b.temperature * t + a.temperature * (1.0f - t);
        intrp.location    = b.location    * t + a.location    * (1.0f - t);

        intrp.rotation = quatToTaitBryat(glm::slerp(taitBryatToQuat(a.rotation), taitBryatToQuat(b.rotation), t));

        return intrp;
    };

    void print();
};
