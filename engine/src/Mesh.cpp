#include <engine/Mesh.h>

#include <utility>

namespace engine {

Mesh::Mesh(const std::vector<float>& vertices,
           const std::vector<unsigned int>& indices,
           const std::vector<int>& layout)
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    int stride = 0;
    for (int size : layout) //foreach int size in layout we add it to the stride
        stride += size;

    int offset = 0;
    for (int i = 0; i < (int)layout.size(); ++i) //for each atrribute we setup
    {
        glVertexAttribPointer(i, layout[i], GL_FLOAT, GL_FALSE,
                              stride * sizeof(float), (void*)(offset * sizeof(float)));
        glEnableVertexAttribArray(i);
        offset += layout[i];
    }

    glBindVertexArray(0); //bind the VAO
    count = (int)indices.size();
}

Mesh::Mesh(int vertexCapacity, const std::vector<int>& layout)
{
    int stride = 0;
    for (int size : layout)
        stride += size;

    capacity = vertexCapacity > 0 ? vertexCapacity : 0;
    floatsPerVertex = stride;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (size_t)capacity * stride * sizeof(float), nullptr,
                 GL_DYNAMIC_DRAW);

    int offset = 0;
    for (int i = 0; i < (int)layout.size(); ++i)
    {
        glVertexAttribPointer(i, layout[i], GL_FLOAT, GL_FALSE,
                              stride * sizeof(float), (void*)(offset * sizeof(float)));
        glEnableVertexAttribArray(i);
        offset += layout[i];
    }

    glBindVertexArray(0);
}

Mesh::~Mesh()
{
    release();
}

Mesh::Mesh(Mesh&& other) noexcept
    : VAO(other.VAO), VBO(other.VBO), EBO(other.EBO), count(other.count),
      capacity(other.capacity), floatsPerVertex(other.floatsPerVertex)
{
    other.VAO = other.VBO = other.EBO = 0;
    other.count = 0;
    other.capacity = other.floatsPerVertex = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept
{
    if (this != &other)
    {
        release();
        VAO = other.VAO;
        VBO = other.VBO;
        EBO = other.EBO;
        count = other.count;
        capacity = other.capacity;
        floatsPerVertex = other.floatsPerVertex;
        other.VAO = other.VBO = other.EBO = 0;
        other.count = 0;
        other.capacity = other.floatsPerVertex = 0;
    }
    return *this;
}

void Mesh::draw(GLenum mode) const
{
    glBindVertexArray(VAO);
    glDrawElements(mode, count, GL_UNSIGNED_INT, 0);
}

void Mesh::setVertices(const float* data, size_t floatCount, size_t floatOffset)
{
    size_t capacityFloats = (size_t)capacity * floatsPerVertex;
    if (VBO == 0 || floatCount == 0 || floatOffset + floatCount > capacityFloats)
        return;

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, floatOffset * sizeof(float),
                    floatCount * sizeof(float), data);
}

void Mesh::drawArrays(GLenum mode, int vertexCount, int first) const
{
    if (vertexCount <= 0)
        return;
    glBindVertexArray(VAO);
    glDrawArrays(mode, first, vertexCount);
}

void Mesh::release()
{
    if (VAO != 0)
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        if (EBO != 0)              // dynamic meshes carry no index buffer
            glDeleteBuffers(1, &EBO);
        VAO = VBO = EBO = 0;
        count = capacity = floatsPerVertex = 0;
    }
}

} // namespace engine
