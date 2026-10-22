#ifndef ENGINE_LATEX_BOARD_H
#define ENGINE_LATEX_BOARD_H

#include <engine/LatexImage.h>
#include <string>
#include <vector>

namespace engine {

// Pages of LaTeX from a text file, shown as a centered white card over the scene: the
// equations of a talk inside the sim, so presenting needs no switch to slides.
// File format: one formula per line (math mode; prose as \text{...}), each centered on
// its own row; a blank line is vertical space; a line "---" starts the next page; lines
// starting with % are comments. The file is read on every open(), so it can be edited
// while the app runs. App owns one and points it at sims/<name>/board.tex.
//
// Owns GL textures (LatexImage): open()/draw() only with the GL context live.
class LatexBoard {
public:
    // The file to read on the next open(). Closes the board.
    void setFile(const std::string& path);

    void open();     // (re)reads the file, keeps the page if it still exists
    void close() { shown = false; }
    void toggle() { shown ? close() : open(); }
    bool visible() const { return shown; }

    void nextPage();
    void prevPage();

    // The card, centered in the display, when visible. Call between ImGui::NewFrame and
    // ImGui::Render. Text size follows the display height, so it reads the same on a
    // laptop and a projector.
    void draw();

private:
    void typesetPage(float size);

    std::string file;
    std::vector<std::vector<std::string>> pages;   // raw lines per page
    std::string readError;                         // file missing / empty
    int page = 0;
    bool shown = false;

    std::vector<LatexImage> lines;                 // the current page, typeset
    std::vector<std::string> lineErrors;           // per line, empty when it parsed
    int typesetPageIndex = -1;
    float typesetSize = 0.0f;
};

} // namespace engine

#endif
