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

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);

    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
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

bool Window::mouseDown(int button) const
{
    return glfwGetMouseButton(window, button) == GLFW_PRESS;
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
