#ifndef ENGINE_SHADER_H
#define ENGINE_SHADER_H

#include <string>
#include <glm.hpp>

namespace engine {

// Compiled + linked GLSL program. Movable, non-copyable; deletes the program on destruction.
class Shader {
public:
    unsigned int ID = 0;

    Shader() = default; // invalid shader (ID 0), assign a real one later
    Shader(const char* vertexPath, const char* fragmentPath);

    // Compile directly from in-memory sources (used by the function-parser codegen path).
    static Shader fromSources(const std::string& vertexSrc, const std::string& fragmentSrc);

    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    void use() const;

    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setVec2(const std::string& name, float x, float y) const;
    void setVec2(const std::string& name, const glm::vec2& value) const;
    void setVec3(const std::string& name, float x, float y, float z) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;
    void setVec4(const std::string& name, float x, float y, float z, float w) const;
    void setVec4(const std::string& name, const glm::vec4& value) const;
    void setMat4(const std::string& name, const glm::mat4& mat) const;

private:
    void compile(const char* vertexSrc, const char* fragmentSrc);
    static void checkCompileErrors(unsigned int shader, const std::string& type);
};

} // namespace engine

#endif
