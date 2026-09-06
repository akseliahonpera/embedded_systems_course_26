#pragma once

#include "Transform.hpp"

struct Camera
{
    Camera(int viewport_width, int viewport_height);

    Camera(const Camera &other) {
        transform = std::make_shared<Transform>();
        vp_width = other.vp_width;
        vp_height = other.vp_height;
        fov = other.fov;
        projection_matrix = other.projection_matrix;

        *transform = *other.transform;
    }
    friend void swap(Camera &a, Camera &b) {
        std::swap(*a.transform, *b.transform);
        std::swap(a.vp_width, b.vp_width);
        std::swap(a.vp_height, b.vp_height);
        std::swap(a.fov, b.fov);
        std::swap(a.projection_matrix, b.projection_matrix);
    }

    Camera &operator= (Camera other) {
        swap(*this, other);
        return *this;
    }

    void setFov(float fov_degrees);
    void setViewPort(int vw, int vh);


    std::shared_ptr<Transform> transform;
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;

private:
    void updateProjectionMatrix();
    float vp_width;
    float vp_height;
    float fov;
    glm::mat4 projection_matrix;
};
