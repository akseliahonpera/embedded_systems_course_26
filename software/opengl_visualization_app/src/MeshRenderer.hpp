#pragma once

#include "includes.hpp"
#include "Mesh.hpp"
#include "Camera.hpp"

struct MeshRenderer
{
protected:
    MeshRenderer();

public:
    static MeshRenderer& getInstance() {
        static MeshRenderer *instance = nullptr;
        if (!instance) {
            instance = new MeshRenderer();
        }
        return *instance;
    }

    MeshRenderer(MeshRenderer &other) = delete;
    void operator=(const MeshRenderer&) = delete;

    void render(const Camera &cam, Mesh &m);

private:
    std::string vertex_src_text;
    std::string fragment_src_text;

    GLuint vertex_shader;
    GLuint fragment_shader;
    GLuint shader_program;

    GLint loc_model_mat;
    GLint loc_view_mat;
    GLint loc_proj_mat;
    GLint loc_normal_mat;
    GLint loc_albedo;
    GLint loc_light_pos;
    GLint loc_light_col;
    GLint loc_camera_pos;
};
