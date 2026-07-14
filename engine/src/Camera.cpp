#include <engine/Camera.h>

#include <GLFW/glfw3.h>
#include <gtc/matrix_transform.hpp>

namespace engine {

void Camera::processInput(GLFWwindow* window, float dt)
{
    if (!inputEnabled)
        return;

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
    // proximity damping: movement scales with how close the radius is to its
    // minimum, so approach is smooth and the camera slows for close inspection
    // (floor of 10% keeps it possible to rotate and back away at the surface)
    if (orbitSlowZone > 0.0f)
        cameraSpeed *= glm::clamp((orbitRadius - orbitRadiusMin) / orbitSlowZone, 0.1f, 1.0f);

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
        orbitRadius -= cameraSpeed;
    orbitRadius = glm::max(orbitRadius, orbitRadiusMin);

    // clamp phi away from poles to avoid lookAt singularity
    const float phiMin = 0.05f;
    const float phiMax = glm::pi<float>() - 0.05f;
    orbitPhi = glm::clamp(orbitPhi, phiMin, phiMax);

    position.x = orbitCenter.x + orbitRadius * sin(orbitPhi) * sin(orbitTheta);
    position.y = orbitCenter.y + orbitRadius * cos(orbitPhi);
    position.z = orbitCenter.z + orbitRadius * sin(orbitPhi) * cos(orbitTheta);
    front = glm::normalize(orbitCenter - position);
    // orbit owns its whole basis: reset up so a mode that tilted it (e.g. a
    // sim's follow cam riding a surface normal) can't leave orbit rolled
    up = glm::vec3(0.0f, 1.0f, 0.0f);
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
