#include <engine/Camera.h>

#include <GLFW/glfw3.h>
#include <gtc/matrix_transform.hpp>

namespace engine {

void Camera::processInput(GLFWwindow* window, float dt)
{
    float cameraSpeed = speed * dt;

    if (mode == Mode::Orbit)
        orbitMode(window, cameraSpeed);
    if (mode == Mode::Free)
        freeMode(window, cameraSpeed);
}

void Camera::freeMode(GLFWwindow* window, float cameraSpeed)
{
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position += cameraSpeed * front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position -= cameraSpeed * front;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position -= glm::normalize(glm::cross(front, up)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position += glm::normalize(glm::cross(front, up)) * cameraSpeed;

    //TODO free rotation
}

void Camera::orbitMode(GLFWwindow* window, float cameraSpeed)
{
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        orbitTheta -= cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        orbitTheta += cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        orbitPhi -= cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        orbitPhi += cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        orbitRadius += cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        orbitRadius = glm::max(orbitRadius - cameraSpeed, 0.1f);

    // clamp phi away from poles to avoid lookAt singularity
    const float phiMin = 0.05f;
    const float phiMax = glm::pi<float>() - 0.05f;
    orbitPhi = glm::clamp(orbitPhi, phiMin, phiMax);

    position.x = orbitCenter.x + orbitRadius * sin(orbitPhi) * sin(orbitTheta);
    position.y = orbitCenter.y + orbitRadius * cos(orbitPhi);
    position.z = orbitCenter.z + orbitRadius * sin(orbitPhi) * cos(orbitTheta);
    front = glm::normalize(orbitCenter - position);
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::lookAt(position, position + front, up);
}

glm::mat4 Camera::getProjectionMatrix() const
{
    return glm::perspective(glm::radians(fovDeg), aspect, nearPlane, farPlane);
}

} // namespace engine
