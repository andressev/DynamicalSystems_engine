#include <engine/FlowKeys.h>

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

bool FlowKeys::handle(float& timeScale, bool& reverse, float dt, float maxScale,
                      float minScale)
{
    if (ImGui::GetIO().WantCaptureKeyboard)
        return false;
    const float before = timeScale;
    const bool wasReversed = reverse;

    if (minScale > 0.0f)
    {
        // logarithmic: a decade per kDecadeSeconds of holding, either way
        const float k = std::log(10.0f) / kDecadeSeconds;
        timeScale = std::clamp(timeScale, minScale, maxScale);
        if (ImGui::IsKeyDown(ImGuiKey_UpArrow))
            timeScale = std::min(maxScale, timeScale * std::exp(k * dt));
        if (ImGui::IsKeyDown(ImGuiKey_DownArrow))
            timeScale = std::max(minScale, timeScale * std::exp(-k * dt));
    }
    else
    {
        // held, not pressed: the speed ramps at kRate per second below 1x (0 -> 1 in
        // ~2 s) and in proportion to itself above, so the top of the range is reachable
        float rate = std::max(kRate, kRate * timeScale);
        if (ImGui::IsKeyDown(ImGuiKey_UpArrow))
            timeScale = std::min(maxScale, timeScale + rate * dt);
        if (ImGui::IsKeyDown(ImGuiKey_DownArrow))
            timeScale = std::max(0.0f, timeScale - rate * dt);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false))
        reverse = true;
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, false))
        reverse = false;

    return timeScale != before || reverse != wasReversed;
}

FlowKeys::TapHold FlowKeys::tapHold(int imguiKey, float dt, float holdSeconds)
{
    // per-key: seconds down so far, and this frame's answer (cached by frame number)
    struct State {
        float downFor = 0.0f;
        int frame = -1;
        TapHold result;
    };
    static State states[ImGuiKey_NamedKey_COUNT];
    const int index = imguiKey - ImGuiKey_NamedKey_BEGIN;
    if (index < 0 || index >= ImGuiKey_NamedKey_COUNT)
        return {};
    State& s = states[index];
    const int frame = ImGui::GetFrameCount();
    if (s.frame == frame)
        return s.result;
    s.frame = frame;

    TapHold r;
    const ImGuiKey key = (ImGuiKey)imguiKey;
    if (ImGui::IsKeyDown(key))
    {
        const float before = s.downFor;
        s.downFor += dt;
        r.held = before < holdSeconds && s.downFor >= holdSeconds;
    }
    else
    {
        if (ImGui::IsKeyReleased(key))
            r.tapped = s.downFor < holdSeconds;
        s.downFor = 0.0f;
    }
    s.result = r;
    return r;
}

void FlowKeys::drawStatus(float timeScale, bool reverse, bool playing)
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                                   vp->WorkPos.y + vp->WorkSize.y - 12.0f),
                            ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.35f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##flowStatus", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    const float alpha = playing ? 1.0f : 0.45f;
    char speed[32];
    // enough digits that a very slow flow doesn't read as 0.00x
    std::snprintf(speed, sizeof(speed),
                  timeScale > 0.0f && timeScale < 0.01f ? "%.4fx"
                  : timeScale > 0.0f && timeScale < 0.1f ? "%.3fx" : "%.2fx",
                  timeScale);
    const char* direction = reverse ? "reversed" : "normal";

    // right-aligned, the speed a size up
    ImGui::SetWindowFontScale(1.5f);
    float w0 = ImGui::CalcTextSize(speed).x;
    ImGui::SetWindowFontScale(1.0f);
    float w1 = ImGui::CalcTextSize(direction).x;
    float width = std::max(w0, w1);

    ImGui::SetWindowFontScale(1.5f);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + width - w0);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, alpha), "%s", speed);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::SetCursorPosX(ImGui::GetStyle().WindowPadding.x + width - w1);
    ImGui::TextColored(reverse ? ImVec4(1.0f, 0.45f, 0.35f, alpha) : ImVec4(0.75f, 0.75f, 0.8f, alpha),
                       "%s", direction);
    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace engine
