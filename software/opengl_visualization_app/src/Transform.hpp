#pragma once

#include "includes.hpp"

struct Transform
{
    Transform();

    void setParent(std::shared_ptr<Transform> other);

    glm::mat4 getModelMatrix() const;
    glm::vec3 getWorldPosition() const;

    glm::vec3 position;
    glm::vec3 rotation;
    glm::vec3 scale;

private:
    std::weak_ptr<Transform> parent;
};
