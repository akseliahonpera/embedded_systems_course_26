#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace ObjLoader
{

struct ObjModel
{
    ObjModel() {
        num_of_faces = 0;
    }

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;

    std::vector<int> faces;

    int num_of_faces;
};

ObjModel loadObj(std::string filename);

}
