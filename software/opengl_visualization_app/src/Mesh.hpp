#pragma once

#include "includes.hpp"
#include "ObjLoader.hpp"
#include "Transform.hpp"

template<typename T>
struct VertexBuffer
{
    VertexBuffer() {
        glCreateBuffers(1, &device_buffer);
    }
    VertexBuffer(const VertexBuffer<T> &other) {
        glCreateBuffers(1, &device_buffer);
        host_buffer = other.host_buffer;
        if (host_buffer.size() > 0) {
            toDevice();
        }
    }

    ~VertexBuffer() {
        glDeleteBuffers(1, &device_buffer);
    }

    friend void swap(VertexBuffer<T> &a, VertexBuffer<T> &b) {
        std::swap(a.device_buffer, b.device_buffer);
        std::swap(a.host_buffer, b.host_buffer);
    }

    VertexBuffer<T> operator= (VertexBuffer<T> other) {
        swap(*this, other);
        return *this;
    }

    void bind(int attribute_index) {
        glBindBuffer(GL_ARRAY_BUFFER, device_buffer);
        glVertexAttribPointer(attribute_index, sizeof(T)/sizeof(float), GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(attribute_index);
    }
    void toDevice() {
        glBindBuffer(GL_ARRAY_BUFFER, device_buffer);
        glBufferData(GL_ARRAY_BUFFER, host_buffer.size()*sizeof(T), host_buffer.data(), GL_STATIC_DRAW);
    }

    std::vector<T> host_buffer;
    GLuint device_buffer;
};


struct Mesh
{
    Mesh() {
        transform = std::make_shared<Transform>();
    }
    Mesh(const Mesh &other) {
        transform = std::make_shared<Transform>();
        vertices = other.vertices;
        uvs = other.uvs;
        normals = other.normals;
        *transform = *other.transform;
    }
    friend void swap(Mesh &a, Mesh &b) {
        std::swap(*a.transform, *b.transform);
        swap(a.vertices, b.vertices);
        swap(a.uvs, b.uvs);
        swap(a.normals, b.normals);
    }

    Mesh &operator= (Mesh other) {
        swap(*this, other);
        return *this;
    }

    static Mesh fromObj(const ObjLoader::ObjModel &model);

    std::shared_ptr<Transform> transform;

    VertexBuffer<glm::vec3> vertices;
    VertexBuffer<glm::vec2> uvs;
    VertexBuffer<glm::vec3> normals;
};
