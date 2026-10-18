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

    // Borderless full screen on the monitor the window sits on (its current video
    // mode: no mode switch, no title bar, no task bar). The title-bar maximize button
    // enters it too; leaving it restores the windowed size and position from before.
    void setFullscreen(bool on);
    void toggleFullscreen() { setFullscreen(!fullscreenOn); }
    bool fullscreen() const { return fullscreenOn; }

private:
    GLFWwindow* window = nullptr;
    int fbWidth = 0;
    int fbHeight = 0;
    bool fullscreenOn = false;
    bool wantFullscreen = false;   // from the maximize callback, applied in pollEvents
    int savedX = 0, savedY = 0, savedW = 0, savedH = 0;   // the windowed geometry

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void maximizeCallback(GLFWwindow* window, int maximized);
};

} // namespace engine

#endif
