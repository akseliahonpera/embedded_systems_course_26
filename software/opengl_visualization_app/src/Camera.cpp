#include "Camera.hpp"
#include <glm/matrix.hpp>

Camera::Camera(int viewport_width, int viewport_height)
{
    transform = std::make_shared<Transform>();

    fov = 90.0f;
    vp_width = static_cast<float>(viewport_width);
    vp_height = static_cast<float>(viewport_height);

    updateProjectionMatrix();
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::inverse(transform->getModelMatrix());
}

glm::mat4 Camera::getProjectionMatrix() const
{
    return projection_matrix;
}

void Camera::setFov(float fov_degrees)
{
    fov = fov_degrees;
    updateProjectionMatrix();
}

void Camera::setViewPort(int vw, int vh)
{
    vp_width = static_cast<float>(vw);
    vp_height = static_cast<float>(vh);

    updateProjectionMatrix();
}

void Camera::updateProjectionMatrix()
{
    float aspect_ratio = vp_width / vp_height;
    projection_matrix = glm::perspective(glm::radians(fov), aspect_ratio, 0.1f, 100.0f);
}
