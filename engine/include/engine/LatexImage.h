#ifndef ENGINE_LATEX_IMAGE_H
#define ENGINE_LATEX_IMAGE_H

#include <glm.hpp>
#include <string>

namespace engine {

// A LaTeX math string typeset into a GL texture. MicroTeX does the TeX layout; the
// result is rasterized on the CPU (stb_truetype for glyphs, coverage-AA rules) and
// uploaded once, so drawing it is just a textured quad — ImGui::Image via draw().
// Re-typeset only when the string changes: set() costs a layout plus a raster.
// See decision 0007-latex-labels-microtex-imgui.
//
// Owns a GL texture: build/set() only after the GL context exists.
class LatexImage {
public:
    LatexImage() = default;
    ~LatexImage();

    LatexImage(const LatexImage&) = delete;
    LatexImage& operator=(const LatexImage&) = delete;
    LatexImage(LatexImage&& other) noexcept;
    LatexImage& operator=(LatexImage&& other) noexcept;

    // Typeset `latex` (math mode, UTF-8) at `textSize` pixels per em in `color`.
    // On a parse error the previous image is kept and error() says why.
    bool set(const std::string& latex, float textSize = 22.0f,
             const glm::vec3& color = glm::vec3(1.0f));

    // ImGui::Image of the formula at its natural pixel size (call inside a window).
    void draw() const;

    // Same image with its top-left at screen point (x, y) in ImGui display pixels, on
    // the background draw list: over the 3D scene, under every ImGui window. For labels
    // that follow something in the world — project it first (Camera matrices).
    void drawAt(float x, float y, float alpha = 1.0f) const;

    bool empty() const { return textureId == 0; }
    unsigned texture() const { return textureId; }
    int width() const { return w; }
    int height() const { return h; }

    // For composing several images into one line of math (e.g. a static part plus
    // per-character pieces of a number that changes every frame, so nothing is
    // re-typeset per frame): the image is the formula's box plus kPad pixels on every
    // side. baseline() is the TeX baseline in pixels from the image top; advance() is
    // the box width, i.e. where the next piece's box starts (image left + kPad).
    static constexpr int kPad = 2;
    float baseline() const { return base; }
    float advance() const { return adv; }
    const std::string& source() const { return src; }
    const std::string& error() const { return err; }

private:
    unsigned textureId = 0;
    int w = 0, h = 0;
    float base = 0.0f, adv = 0.0f;
    std::string src;
    std::string err;
};

} // namespace engine

#endif
