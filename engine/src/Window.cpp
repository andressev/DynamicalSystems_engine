#include <engine/Window.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stdexcept>

namespace engine {

Window::Window(const char* title, int width, int height)
{
    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // 4x MSAA: without it points and thin lines snap to whole pixels, so anything
    // moving less than a pixel per frame crawls in visible one-pixel jumps
    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync: even frame pacing, no tearing on moving geometry

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glEnable(GL_MULTISAMPLE);

    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);

    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetWindowMaximizeCallback(window, maximizeCallback);
}

Window::~Window()
{
    if (window)
        glfwDestroyWindow(window);
    glfwTerminate();
}

bool Window::shouldClose() const
{
    return glfwWindowShouldClose(window);
}

void Window::requestClose()
{
    glfwSetWindowShouldClose(window, true);
}

void Window::swapBuffers()
{
    glfwSwapBuffers(window);
}

void Window::pollEvents()
{
    glfwPollEvents();
    if (wantFullscreen)
    {
        wantFullscreen = false;
        setFullscreen(true);
    }
}

void Window::setFullscreen(bool on)
{
    if (on == fullscreenOn)
        return;
    if (on)
    {
        // the maximize button brings us here maximized: restore first so the saved
        // geometry (and the window we come back to) is the plain windowed one
        if (glfwGetWindowAttrib(window, GLFW_MAXIMIZED))
            glfwRestoreWindow(window);
        glfwGetWindowPos(window, &savedX, &savedY);
        glfwGetWindowSize(window, &savedW, &savedH);
        // the monitor holding the window's centre (the primary if none does)
        GLFWmonitor* target = glfwGetPrimaryMonitor();
        int count = 0;
        GLFWmonitor** monitors = glfwGetMonitors(&count);
        const int cx = savedX + savedW / 2, cy = savedY + savedH / 2;
        for (int i = 0; i < count; ++i)
        {
            int mx, my;
            glfwGetMonitorPos(monitors[i], &mx, &my);
            const GLFWvidmode* m = glfwGetVideoMode(monitors[i]);
            if (m && cx >= mx && cx < mx + m->width && cy >= my && cy < my + m->height)
                target = monitors[i];
        }
        const GLFWvidmode* mode = glfwGetVideoMode(target);
        if (!mode)
            return;
        // the monitor's own mode = "windowed full screen": no mode switch, nothing else on screen
        glfwSetWindowMonitor(window, target, 0, 0, mode->width, mode->height, mode->refreshRate);
        fullscreenOn = true;
    }
    else
    {
        glfwSetWindowMonitor(window, nullptr, savedX, savedY, savedW, savedH, 0);
        fullscreenOn = false;
    }
}

void Window::maximizeCallback(GLFWwindow* window, int maximized)
{
    Window* self = (Window*)glfwGetWindowUserPointer(window);
    if (self && maximized && !self->fullscreenOn)
        self->wantFullscreen = true;   // not from inside the callback: at the next pollEvents
}

glm::vec2 Window::cursorPosNormalized() const
{
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    int w, h;
    glfwGetWindowSize(window, &w, &h);
    if (w <= 0 || h <= 0)
        return glm::vec2(0.0f);
    return glm::vec2((float)mx / w, 1.0f - (float)my / h);
}

bool Window::keyDown(int key) const
{
    return glfwGetKey(window, key) == GLFW_PRESS;
}

void Window::framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    Window* self = (Window*)glfwGetWindowUserPointer(window);
    if (self)
    {
        self->fbWidth = width;
        self->fbHeight = height;
    }
}

} // namespace engine
