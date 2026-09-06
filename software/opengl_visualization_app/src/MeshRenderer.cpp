#include "Utils.hpp"
#include "MeshRenderer.hpp"
#include <glm/gtc/type_ptr.hpp>

MeshRenderer::MeshRenderer() {
    vertex_src_text = Utils::readTextFile("shaders/vertex.glsl");
    fragment_src_text = Utils::readTextFile("shaders/fragment.glsl");

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    const char *v_src = vertex_src_text.c_str();
    const char *f_src = fragment_src_text.c_str();

    glShaderSource(vertex_shader, 1, (const GLchar**)&v_src, 0);
    glShaderSource(fragment_shader, 1, (const GLchar**)&f_src, 0);

    glCompileShader(vertex_shader);
    glCompileShader(fragment_shader);

    shader_program = glCreateProgram();

    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);

    glBindAttribLocation(shader_program, 0, "aPosition");
    glBindAttribLocation(shader_program, 1, "aNormal");
    glBindAttribLocation(shader_program, 2, "aTexCoord");

    glLinkProgram(shader_program);



    loc_model_mat  = glGetUniformLocation(shader_program, "uModel");
    loc_view_mat   = glGetUniformLocation(shader_program, "uView");
    loc_proj_mat   = glGetUniformLocation(shader_program, "uProjection");
    loc_normal_mat = glGetUniformLocation(shader_program, "uNormalMatrix");
    //loc_albedo     = glGetUniformLocation(shader_program, "uAlbedo");
    loc_light_pos  = glGetUniformLocation(shader_program, "uLightPos");
    loc_light_col  = glGetUniformLocation(shader_program, "uLightColor");
    loc_camera_pos = glGetUniformLocation(shader_program, "uCameraPos");

}


void MeshRenderer::render(const Camera &cam, Mesh &m)
{
    glUseProgram(shader_program);

    glm::mat4 model = m.transform->getModelMatrix();
    glm::mat4 proj  = cam.getProjectionMatrix();
    glm::mat4 view  = cam.getViewMatrix();
    glm::vec3 cam_pos = cam.transform->getWorldPosition();

    glm::mat3 normal_mat = model;

    glUniformMatrix4fv(loc_model_mat, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(loc_view_mat,  1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(loc_proj_mat,  1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix3fv(loc_normal_mat, 1, GL_FALSE, glm::value_ptr(normal_mat));
    glUniform3f(loc_light_pos, 2.0f, 3.0f, 2.5f);
    glUniform3f(loc_light_col, 1.0f, 0.97f, 0.92f);
    glUniform3f(loc_camera_pos, cam_pos.x, cam_pos.y, cam_pos.z);

    m.vertices.bind(0);
    m.normals.bind(1);
    m.uvs.bind(2);

    glDrawArrays(GL_TRIANGLES, 0, m.vertices.host_buffer.size());
}





