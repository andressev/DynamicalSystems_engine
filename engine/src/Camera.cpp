#include <engine/Camera.h>

#include <GLFW/glfw3.h>
#include <gtc/matrix_transform.hpp>
#include <cmath>

namespace engine {

void Camera::processInput(GLFWwindow* window, float dt)
{
    if (!inputEnabled)
        return;

    if (mode == Mode::Orbit)
        orbitMode(window, dt);
    if (mode == Mode::Free)
        freeMode(window, speed * dt);
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

void Camera::orbitMode(GLFWwindow* window, float dt)
{
    // rotation is in radians, zoom in world units — two rates, one damping factor
    float turn = orbitAngularSpeed * dt;
    float zoom = speed * dt;

    // proximity damping: movement scales with how close the radius is to its
    // minimum, so approach is smooth and the camera slows for close inspection
    // (floor of 10% keeps it possible to rotate and back away at the surface)
    if (orbitSlowZone > 0.0f)
    {
        float damp = glm::clamp((orbitRadius - orbitRadiusMin) / orbitSlowZone, 0.1f, 1.0f);
        turn *= damp;
        zoom *= damp;
    }

    for (int key : {GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_W, GLFW_KEY_S, GLFW_KEY_E, GLFW_KEY_Q})
        if (glfwGetKey(window, key) == GLFW_PRESS)
            cancelEase(); // the user takes over mid-glide

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        orbitTheta -= turn;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        orbitTheta += turn;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        orbitPhi -= turn;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        orbitPhi += turn;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        orbitRadius += zoom;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        orbitRadius -= zoom;
    orbitRadius = glm::max(orbitRadius, orbitRadiusMin);

    // clamp phi away from poles to avoid lookAt singularity
    const float phiMin = 0.05f;
    const float phiMax = glm::pi<float>() - 0.05f;
    orbitPhi = glm::clamp(orbitPhi, phiMin, phiMax);

    placeOnOrbit();
}

void Camera::placeOnOrbit()
{
    position.x = orbitCenter.x + orbitRadius * sin(orbitPhi) * sin(orbitTheta);
    position.y = orbitCenter.y + orbitRadius * cos(orbitPhi);
    position.z = orbitCenter.z + orbitRadius * sin(orbitPhi) * cos(orbitTheta);
    front = glm::normalize(orbitCenter - position);
}

void Camera::easeOrbitTo(const glm::vec3& center, float radius, float theta, float phi,
                         float seconds)
{
    easeT = 0.0f;
    easeDuration = glm::max(seconds, 1e-3f);
    easeC0 = orbitCenter;
    easeC1 = center;
    easeR0 = glm::max(orbitRadius, 1e-4f);
    easeR1 = glm::max(radius, 1e-4f);
    easeTh0 = orbitTheta;
    float dTheta = theta - orbitTheta; // shortest way round
    dTheta -= glm::two_pi<float>() * std::round(dTheta / glm::two_pi<float>());
    easeTh1 = orbitTheta + dTheta;
    easePh0 = orbitPhi;
    easePh1 = phi;
}

void Camera::updateEase(float dt)
{
    if (!easing())
        return;
    easeT += dt;
    const float t = glm::clamp(easeT / easeDuration, 0.0f, 1.0f);
    const float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); // smootherstep
    orbitCenter = glm::mix(easeC0, easeC1, s);
    orbitRadius = easeR0 * std::pow(easeR1 / easeR0, s);
    orbitTheta = glm::mix(easeTh0, easeTh1, s);
    orbitPhi = glm::mix(easePh0, easePh1, s);
    if (t >= 1.0f)
        easeDuration = 0.0f;
    // orbit input may be skipped this frame (ImGui has the keyboard): place anyway
    if (mode == Mode::Orbit)
        placeOnOrbit();
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
