#include "Mesh.hpp"

Mesh Mesh::fromObj(const ObjLoader::ObjModel &model)
{
    Mesh m;

    bool has_normals = (model.normals.size() > 0);
    bool has_uvs = (model.uvs.size() > 0);

    int indices_per_face = (1 + has_normals + has_uvs)*3;

    if (static_cast<int>(model.faces.size()) != indices_per_face*model.num_of_faces) {
        std::cout << "Error. Unsupported obj. Only triangles are allowed. Make sure to enable triangulation in Blender" << std::endl;
    }

    size_t read_idx = 0;
    for (int i = 0; i < model.num_of_faces; i++) {
        for (int j = 0; j < 3; j++) {
            int vertex_idx = model.faces[read_idx++];
            m.vertices.host_buffer.push_back(model.vertices[vertex_idx]);

            if (has_uvs) {
                int uv_idx = model.faces[read_idx++];
                m.uvs.host_buffer.push_back(model.uvs[uv_idx]);
            }
            if (has_normals) {
                int normal_idx = model.faces[read_idx++];
                m.normals.host_buffer.push_back(model.normals[normal_idx]);
            }
        }
    }

    m.normals.toDevice();
    m.uvs.toDevice();
    m.vertices.toDevice();

    return m;
}
