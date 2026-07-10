#ifndef ENGINE_MESH_H
#define ENGINE_MESH_H

#include <glad/glad.h>
#include <vector>

namespace engine {

// RAII wrapper around one VAO + VBO + EBO.
// This is the single place vertex buffers live on the GPU
// (future CUDA interop registers the VBO returned by vbo()).
class Mesh {
public:
    // layout = number of floats per attribute, e.g. {3, 3, 2} for pos/normal/uv
    Mesh(const std::vector<float>& vertices,
         const std::vector<unsigned int>& indices,
         const std::vector<int>& layout); //floats per atribute

    ~Mesh();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void draw(GLenum mode = GL_TRIANGLES) const;

    int indexCount() const { return count; }
    unsigned int vbo() const { return VBO; }

private:
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    int count = 0;

    void release();
};

} // namespace engine

#endif
