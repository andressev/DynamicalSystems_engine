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

Mesh::~Mesh()
{
    release();
}

Mesh::Mesh(Mesh&& other) noexcept
    : VAO(other.VAO), VBO(other.VBO), EBO(other.EBO), count(other.count)
{
    other.VAO = other.VBO = other.EBO = 0;
    other.count = 0;
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
        other.VAO = other.VBO = other.EBO = 0;
        other.count = 0;
    }
    return *this;
}

void Mesh::draw(GLenum mode) const
{
    glBindVertexArray(VAO);
    glDrawElements(mode, count, GL_UNSIGNED_INT, 0);
}

void Mesh::release()
{
    if (VAO != 0)
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        VAO = VBO = EBO = 0;
        count = 0;
    }
}

} // namespace engine
