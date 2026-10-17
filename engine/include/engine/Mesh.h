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

    // Instanced drawing: a second, dynamic buffer of per-instance attributes (divisor 1)
    // whose locations continue after the vertex layout's. Re-calling it replaces the
    // buffer (grow by calling it again with a larger capacity).
    void setInstanceLayout(int instanceCapacity, const std::vector<int>& layout);
    // Overwrite the first floatCount floats of the instance buffer (clamped like
    // setVertices: an over-capacity write uploads nothing).
    void setInstances(const float* data, size_t floatCount);
    // Draw the whole mesh `instanceCount` times: indexed when built with indices,
    // otherwise over `vertexCount` vertices.
    void drawInstanced(GLenum mode, int instanceCount, int vertexCount = -1) const;
    int instanceCapacity() const { return instCapacity; }

    int indexCount() const { return count; }
    int vertexCapacity() const { return capacity; }
    unsigned int vbo() const { return VBO; }

private:
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    unsigned int instVBO = 0;
    int count = 0;
    int capacity = 0;        // dynamic meshes: vertices the VBO has room for
    int floatsPerVertex = 0; // stride, in floats
    int attribCount = 0;     // vertex attributes; instance attributes start here
    int instCapacity = 0;
    int floatsPerInstance = 0;

    void release();
};

} // namespace engine

#endif
