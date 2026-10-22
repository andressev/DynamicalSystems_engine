#include <engine/LatexBoard.h>

#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <utility>

namespace engine {

namespace {

constexpr float kSizeFraction = 0.045f;   // text size per display height
const glm::vec3 kInk(0.08f, 0.08f, 0.10f);

std::string trimmed(const std::string& s)
{
    size_t a = s.find_first_not_of(" \t\r");
    if (a == std::string::npos)
        return "";
    size_t b = s.find_last_not_of(" \t\r");
    return s.substr(a, b - a + 1);
}

// blank lines at either end of a page carry no meaning
void stripBlankEnds(std::vector<std::string>& lines)
{
    while (!lines.empty() && lines.back().empty())
        lines.pop_back();
    lines.erase(lines.begin(),
                std::find_if(lines.begin(), lines.end(), [](const std::string& l) { return !l.empty(); }));
}

// a clickable triangle of side ~s pointing left (dir = -1) or right (+1), faded when off
bool arrowButton(const char* id, float dir, bool enabled, float s)
{
    const bool pressed = ImGui::InvisibleButton(id, ImVec2(s, s)) && enabled;
    const float alpha = !enabled ? 0.15f : ImGui::IsItemHovered() ? 1.0f : 0.55f;
    const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
    const ImVec2 c(0.5f * (a.x + b.x), 0.5f * (a.y + b.y));
    const float h = 0.4f * s;
    ImVec2 tip(c.x + dir * h, c.y), p(c.x - dir * h, c.y + h), q(c.x - dir * h, c.y - h);
    if (dir < 0)
        std::swap(p, q);   // keep the winding clockwise for the anti-aliased edge
    ImGui::GetWindowDrawList()->AddTriangleFilled(tip, p, q,
                                                  ImGui::GetColorU32(ImVec4(kInk.r, kInk.g, kInk.b, alpha)));
    return pressed;
}

} // namespace

void LatexBoard::setFile(const std::string& path)
{
    file = path;
    page = 0;
    shown = false;
    typesetPageIndex = -1;
}

void LatexBoard::open()
{
    pages.clear();
    readError.clear();
    typesetPageIndex = -1;
    shown = true;

    std::ifstream in(file);
    if (!in)
    {
        readError = "no " + file;
        return;
    }
    std::vector<std::string> current;
    std::string raw;
    while (std::getline(in, raw))
    {
        const std::string line = trimmed(raw);
        if (line.rfind('%', 0) == 0)
            continue;
        if (line == "---")
        {
            stripBlankEnds(current);
            if (!current.empty())
                pages.push_back(std::move(current));
            current.clear();
            continue;
        }
        current.push_back(line);
    }
    stripBlankEnds(current);
    if (!current.empty())
        pages.push_back(std::move(current));

    if (pages.empty())
        readError = file + " is empty";
    page = std::clamp(page, 0, std::max(0, (int)pages.size() - 1));
}

void LatexBoard::nextPage()
{
    if (page + 1 < (int)pages.size())
        ++page;
}

void LatexBoard::prevPage()
{
    if (page > 0)
        --page;
}

void LatexBoard::typesetPage(float size)
{
    typesetPageIndex = page;
    typesetSize = size;
    lines.clear();
    lineErrors.clear();
    if (page >= (int)pages.size())
        return;
    const std::vector<std::string>& src = pages[page];
    lines.resize(src.size());
    lineErrors.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i)
        if (!src[i].empty() && !lines[i].set(src[i], size, kInk))
            lineErrors[i] = lines[i].error();
}

void LatexBoard::draw()
{
    if (!shown)
        return;

    const ImGuiIO& io = ImGui::GetIO();
    const float size = std::round(io.DisplaySize.y * kSizeFraction);
    if (page != typesetPageIndex || size != typesetSize)
        typesetPage(size);

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(kInk.r, kInk.g, kInk.b, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(size, 0.6f * size));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.25f * size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##latexBoard", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoNav);

    if (!readError.empty())
    {
        ImGui::TextUnformatted(readError.c_str());
    }
    else
    {
        // every row centered on the widest one
        float widest = 0.0f;
        for (const LatexImage& img : lines)
            widest = std::max(widest, (float)img.width());
        const float left = ImGui::GetCursorPosX();
        const std::vector<std::string>& src = pages[page];
        for (size_t i = 0; i < lines.size(); ++i)
        {
            if (src[i].empty())
                ImGui::Dummy(ImVec2(0.0f, 0.6f * size));
            else if (!lineErrors[i].empty() || lines[i].empty())
                ImGui::TextColored(ImVec4(0.8f, 0.1f, 0.1f, 1.0f), "%s  (%s)", src[i].c_str(),
                                   lineErrors[i].c_str());
            else
            {
                ImGui::SetCursorPosX(left + 0.5f * (widest - (float)lines[i].width()));
                lines[i].draw();
            }
        }
        // ◀  p / n  ▶ along the bottom
        if (pages.size() > 1)
        {
            char num[32];
            std::snprintf(num, sizeof(num), "%d / %d", page + 1, (int)pages.size());
            const ImVec2 t = ImGui::CalcTextSize(num);
            const float s = size;
            const float rowW = std::max(widest, 3.0f * s + t.x);
            const float rowY = ImGui::GetCursorPosY() + 0.3f * size;

            ImGui::SetCursorPos(ImVec2(left, rowY));
            if (arrowButton("##prev", -1.0f, page > 0, s))
                prevPage();
            ImGui::SameLine();
            ImGui::SetCursorPos(ImVec2(left + 0.5f * (rowW - t.x), rowY + 0.5f * (s - t.y)));
            ImGui::TextDisabled("%s", num);
            ImGui::SameLine();
            ImGui::SetCursorPos(ImVec2(left + rowW - s, rowY));
            if (arrowButton("##next", 1.0f, page + 1 < (int)pages.size(), s))
                nextPage();
        }
    }

    ImGui::End();
    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(2);
}

} // namespace engine
