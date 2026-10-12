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

    // Dynamic, index-less buffer: reserves room for `vertexCapacity` vertices
    // (GL_DYNAMIC_DRAW, no EBO) for geometry rebuilt at runtime — growing
    // trajectories, glyph grids re-sampled every frame. Fill with setVertices(),
    // draw with drawArrays().
    Mesh(int vertexCapacity, const std::vector<int>& layout);

    ~Mesh();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void draw(GLenum mode = GL_TRIANGLES) const;

    // Overwrite `floatCount` floats starting at `floatOffset` in the VBO. Only for
    // meshes built with the dynamic constructor; writes past the capacity are
    // clamped away (nothing is uploaded).
    void setVertices(const float* data, size_t floatCount, size_t floatOffset = 0);

    // glDrawArrays over the first `vertexCount` vertices starting at `first` —
    // the draw call for dynamic meshes, which carry no index buffer.
    void drawArrays(GLenum mode, int vertexCount, int first = 0) const;

    int indexCount() const { return count; }
    int vertexCapacity() const { return capacity; }
    unsigned int vbo() const { return VBO; }

private:
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    int count = 0;
    int capacity = 0;        // dynamic meshes: vertices the VBO has room for
    int floatsPerVertex = 0; // stride, in floats

    void release();
};

} // namespace engine

#endif
