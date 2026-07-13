#ifndef ENGINE_WINDOW_H
#define ENGINE_WINDOW_H

#include <glm.hpp>

struct GLFWwindow;

namespace engine {

// Owns the GLFW window and the OpenGL context.
// There should be exactly one Window alive at a time (it calls glfwInit/glfwTerminate).
class Window {
public:
    Window(const char* title, int width, int height);
    ~Window();
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool shouldClose() const;
    void requestClose();
    void swapBuffers();
    void pollEvents();

    int width() const { return fbWidth; }
    int height() const { return fbHeight; }
    float aspect() const { return fbHeight > 0 ? (float)fbWidth / (float)fbHeight : 1.0f; }

    // cursor position normalized to [0,1] x [0,1], origin bottom-left
    glm::vec2 cursorPosNormalized() const;

    // true while the key is held; pass an uppercase ASCII character
    // (GLFW key codes match ASCII for letters/digits, e.g. keyDown('W'))
    bool keyDown(int key) const;

    GLFWwindow* handle() const { return window; }

private:
    GLFWwindow* window = nullptr;
    int fbWidth = 0;
    int fbHeight = 0;

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
};

} // namespace engine

#endif
