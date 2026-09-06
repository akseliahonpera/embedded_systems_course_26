#include "Transform.hpp"
#include <glm/gtc/matrix_transform.hpp>

Transform::Transform()
{
    scale = glm::vec3(1.0f, 1.0f, 1.0f);
    rotation = glm::vec3(0.0f, 0.0f, 0.0f);
    position = glm::vec3(0.0f, 0.0f, 0.0f);
}


void Transform::setParent(std::shared_ptr<Transform> other)
{
    parent = other;
}

glm::mat4 Transform::getModelMatrix() const
{
    glm::mat4 mat = glm::mat4(1.0f);

    if (auto p = parent.lock()) {
        mat = p->getModelMatrix();
    }

    mat = glm::translate(mat, position);

    mat = glm::rotate(mat, rotation.x, glm::vec3(1, 0, 0));
    mat = glm::rotate(mat, rotation.y, glm::vec3(0, 1, 0));
    mat = glm::rotate(mat, rotation.z, glm::vec3(0, 0, 1));

    mat = glm::scale(mat, scale);

    return mat;
}

glm::vec3 Transform::getWorldPosition() const
{
    return glm::vec3(getModelMatrix()*glm::vec4(0, 0, 0, 1));
}
