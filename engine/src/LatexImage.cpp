#include <engine/LatexImage.h>

#include <glad/glad.h>
#include <imgui.h>

// MicroTeX: TeX parsing and box layout. Drawing goes through tex::Graphics2D, which
// this file implements as a small CPU rasterizer.
#include "latex.h"
#include "utils/utf.h"

// imgui_draw.cpp compiles its copy with STBTT_STATIC too, so the two never collide
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include <imstb_truetype.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

namespace engine {

namespace {

// ---- fonts ----------------------------------------------------------------------

// One loaded .ttf; shared by every tex::Font made from the same file, kept for the
// life of the process (MicroTeX caches its fonts statically too).
struct FontFile {
    std::vector<unsigned char> bytes;
    stbtt_fontinfo info{};
    bool ok = false;
};

std::shared_ptr<FontFile> loadFontFile(const std::string& path)
{
    static std::map<std::string, std::shared_ptr<FontFile>> cache;
    auto it = cache.find(path);
    if (it != cache.end())
        return it->second;

    auto f = std::make_shared<FontFile>();
    std::ifstream in(path, std::ios::binary);
    f->bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    f->ok = !f->bytes.empty() &&
            stbtt_InitFont(&f->info, f->bytes.data(),
                           stbtt_GetFontOffsetForIndex(f->bytes.data(), 0)) != 0;
    cache[path] = f;
    return f;
}

// Text that is not math (\text{...}) asks for a system family by name. Map those onto
// the Computer Modern text fonts MicroTeX ships, so no system font lookup is needed.
std::string namedFontPath(const std::string& name, int style)
{
    const std::string dir = std::string(MICROTEX_RES_DIR) + "/fonts/latin/";
    bool sans = name.find("Sans") != std::string::npos;
    switch (style)
    {
    case tex::BOLD:       return dir + (sans ? "sb10.ttf" : "bx10.ttf");
    case tex::ITALIC:     return dir + (sans ? "si10.ttf" : "i10.ttf");
    case tex::BOLDITALIC: return dir + (sans ? "sbi10.ttf" : "bi10.ttf");
    default:              return dir + (sans ? "ss10.ttf" : "r10.ttf");
    }
}

class StbFont : public tex::Font {
public:
    StbFont(std::string path, std::string family, float size)
        : path(std::move(path)), family(std::move(family)), size(size),
          file(loadFontFile(this->path))
    {}

    float getSize() const override { return size; }

    tex::sptr<tex::Font> deriveFont(int style) const override
    {
        if (family.empty()) // loaded from a file: there is only the one face
            return std::make_shared<StbFont>(path, family, size);
        return std::make_shared<StbFont>(namedFontPath(family, style), family, size);
    }

    bool operator==(const tex::Font& o) const override
    {
        const auto& f = static_cast<const StbFont&>(o);
        return file == f.file && size == f.size;
    }
    bool operator!=(const tex::Font& o) const override { return !(*this == o); }

    // Glyph for a code point. MicroTeX's TeX fonts put a few glyphs in the
    // private-use symbol range, so fall back to U+F000 + c.
    int glyph(wchar_t c) const
    {
        if (!file->ok)
            return 0;
        int g = stbtt_FindGlyphIndex(&file->info, (int)c);
        if (g == 0 && c < 0x100)
            g = stbtt_FindGlyphIndex(&file->info, 0xF000 + (int)c);
        return g;
    }

    std::string path;
    std::string family;   // empty for fonts created from a file
    float size;
    std::shared_ptr<FontFile> file;
};

// ---- canvas ---------------------------------------------------------------------

// Premultiplied RGBA float image that the Graphics2D below paints into.
struct Canvas {
    int w = 0, h = 0;
    std::vector<float> px; // 4 floats per pixel

    Canvas(int w, int h) : w(w), h(h), px((size_t)w * h * 4, 0.0f) {}

    void blend(int x, int y, float coverage, tex::color c)
    {
        if (x < 0 || y < 0 || x >= w || y >= h || coverage <= 0.0f)
            return;
        float a = std::min(coverage, 1.0f) * (float)tex::color_a(c) / 255.0f;
        float* p = &px[((size_t)y * w + x) * 4];
        float src[3] = {tex::color_r(c) / 255.0f, tex::color_g(c) / 255.0f,
                        tex::color_b(c) / 255.0f};
        for (int i = 0; i < 3; ++i)
            p[i] = src[i] * a + p[i] * (1.0f - a);
        p[3] = a + p[3] * (1.0f - a);
    }
};

// ---- Graphics2D -----------------------------------------------------------------

// Affine transform + painting onto a Canvas. Glyphs honor translation and the
// y scale (MicroTeX's own rule: characters are sized by sy), not rotation — rotated
// math (\rotatebox) is the one thing this backend does not draw faithfully.
class CanvasGraphics : public tex::Graphics2D {
public:
    explicit CanvasGraphics(Canvas& canvas) : canvas(canvas) {}

    void setColor(tex::color c) override { col = c; }
    tex::color getColor() const override { return col; }
    void setStroke(const tex::Stroke& s) override { stroke = s; }
    const tex::Stroke& getStroke() const override { return stroke; }
    void setStrokeWidth(float w) override { stroke.lineWidth = w; }
    const tex::Font* getFont() const override { return font; }
    void setFont(const tex::Font* f) override { font = f; }

    void translate(float dx, float dy) override
    {
        m[4] += m[0] * dx + m[2] * dy;
        m[5] += m[1] * dx + m[3] * dy;
    }
    void scale(float x, float y) override
    {
        m[0] *= x; m[1] *= x;
        m[2] *= y; m[3] *= y;
        sxAcc *= x;
        syAcc *= y;
    }
    void rotate(float angle) override
    {
        float c = std::cos(angle), s = std::sin(angle);
        float a = m[0], b = m[1], cc = m[2], d = m[3];
        m[0] = a * c + cc * s;  m[1] = b * c + d * s;
        m[2] = -a * s + cc * c; m[3] = -b * s + d * c;
    }
    void rotate(float angle, float px, float py) override
    {
        translate(px, py);
        rotate(angle);
        translate(-px, -py);
    }
    void reset() override
    {
        m[0] = 1; m[1] = 0; m[2] = 0; m[3] = 1; m[4] = 0; m[5] = 0;
        sxAcc = syAcc = 1.0f;
    }
    float sx() const override { return sxAcc; }
    float sy() const override { return syAcc; }

    void drawChar(wchar_t c, float x, float y) override
    {
        const StbFont* f = static_cast<const StbFont*>(font);
        if (!f || !f->file->ok)
            return;
        int g = f->glyph(c);
        if (g == 0)
            return;

        float pixelSize = f->size * std::abs(syAcc);
        if (pixelSize < 0.5f)
            return;
        float scale = stbtt_ScaleForMappingEmToPixels(&f->file->info, pixelSize);

        float ox, oy;
        apply(x, y, ox, oy);
        float fx = std::floor(ox), fy = std::floor(oy);
        float shiftX = ox - fx, shiftY = oy - fy;

        int x0, y0, x1, y1;
        stbtt_GetGlyphBitmapBoxSubpixel(&f->file->info, g, scale, scale, shiftX, shiftY,
                                        &x0, &y0, &x1, &y1);
        int bw = x1 - x0, bh = y1 - y0;
        if (bw <= 0 || bh <= 0)
            return;

        glyphBuf.assign((size_t)bw * bh, 0);
        stbtt_MakeGlyphBitmapSubpixel(&f->file->info, glyphBuf.data(), bw, bh, bw, scale,
                                      scale, shiftX, shiftY, g);
        for (int j = 0; j < bh; ++j)
            for (int i = 0; i < bw; ++i)
            {
                unsigned char v = glyphBuf[(size_t)j * bw + i];
                if (v)
                    canvas.blend((int)fx + x0 + i, (int)fy + y0 + j, v / 255.0f, col);
            }
    }

    void drawText(const std::wstring& s, float x, float y) override
    {
        const StbFont* f = static_cast<const StbFont*>(font);
        if (!f || !f->file->ok)
            return;
        float scale = stbtt_ScaleForMappingEmToPixels(&f->file->info, f->size);
        for (wchar_t c : s)
        {
            drawChar(c, x, y);
            int adv, lsb;
            stbtt_GetGlyphHMetrics(&f->file->info, f->glyph(c), &adv, &lsb);
            x += adv * scale; // font units -> user units, before the context scale
        }
    }

    void drawLine(float x1, float y1, float x2, float y2) override
    {
        float ax, ay, bx, by;
        apply(x1, y1, ax, ay);
        apply(x2, y2, bx, by);
        float half = 0.5f * stroke.lineWidth * std::abs(syAcc);
        line(ax, ay, bx, by, half);
    }

    void drawRect(float x, float y, float w, float h) override
    {
        drawLine(x, y, x + w, y);
        drawLine(x + w, y, x + w, y + h);
        drawLine(x + w, y + h, x, y + h);
        drawLine(x, y + h, x, y);
    }

    void fillRect(float x, float y, float w, float h) override
    {
        // TeX rules are axis-aligned: coverage = pixel ∩ rect area, exact AA
        float ax, ay, bx, by;
        apply(x, y, ax, ay);
        apply(x + w, y + h, bx, by);
        float l = std::min(ax, bx), r = std::max(ax, bx);
        float t = std::min(ay, by), b = std::max(ay, by);
        for (int j = (int)std::floor(t); j < (int)std::ceil(b); ++j)
        {
            float cy = std::min(b, j + 1.0f) - std::max(t, (float)j);
            for (int i = (int)std::floor(l); i < (int)std::ceil(r); ++i)
            {
                float cx = std::min(r, i + 1.0f) - std::max(l, (float)i);
                canvas.blend(i, j, cx * cy, col);
            }
        }
    }

    void drawRoundRect(float x, float y, float w, float h, float, float) override
    {
        drawRect(x, y, w, h);
    }
    void fillRoundRect(float x, float y, float w, float h, float, float) override
    {
        fillRect(x, y, w, h);
    }

private:
    void apply(float x, float y, float& ox, float& oy) const
    {
        ox = m[0] * x + m[2] * y + m[4];
        oy = m[1] * x + m[3] * y + m[5];
    }

    // thick segment, coverage from distance to the segment (1px soft edge)
    void line(float ax, float ay, float bx, float by, float half)
    {
        float l = std::min(ax, bx) - half - 1, r = std::max(ax, bx) + half + 1;
        float t = std::min(ay, by) - half - 1, b = std::max(ay, by) + half + 1;
        float dx = bx - ax, dy = by - ay;
        float len2 = dx * dx + dy * dy;
        for (int j = (int)std::floor(t); j <= (int)std::ceil(b); ++j)
            for (int i = (int)std::floor(l); i <= (int)std::ceil(r); ++i)
            {
                float px = i + 0.5f - ax, py = j + 0.5f - ay;
                float u = len2 > 0 ? std::clamp((px * dx + py * dy) / len2, 0.0f, 1.0f) : 0.0f;
                float ex = px - u * dx, ey = py - u * dy;
                float d = std::sqrt(ex * ex + ey * ey);
                canvas.blend(i, j, std::clamp(half + 0.5f - d, 0.0f, 1.0f), col);
            }
    }

    Canvas& canvas;
    float m[6] = {1, 0, 0, 1, 0, 0}; // column-major 2x3: [a b c d tx ty]
    float sxAcc = 1.0f, syAcc = 1.0f;
    tex::color col = tex::black;
    tex::Stroke stroke;
    const tex::Font* font = nullptr;
    std::vector<unsigned char> glyphBuf;
};

class StbTextLayout : public tex::TextLayout {
public:
    StbTextLayout(std::wstring text, tex::sptr<StbFont> font)
        : text(std::move(text)), font(std::move(font))
    {}

    void getBounds(tex::Rect& r) override
    {
        r = tex::Rect();
        if (!font->file->ok)
            return;
        const stbtt_fontinfo* info = &font->file->info;
        float scale = stbtt_ScaleForMappingEmToPixels(info, font->size);
        int asc, desc, gap;
        stbtt_GetFontVMetrics(info, &asc, &desc, &gap);
        float width = 0.0f;
        for (wchar_t c : text)
        {
            int adv, lsb;
            stbtt_GetGlyphHMetrics(info, font->glyph(c), &adv, &lsb);
            width += adv * scale;
        }
        r.x = 0.0f;
        r.y = -asc * scale;
        r.w = width;
        r.h = (asc - desc) * scale;
    }

    void draw(tex::Graphics2D& g2, float x, float y) override
    {
        const tex::Font* prev = g2.getFont();
        g2.setFont(font.get());
        g2.drawText(text, x, y);
        g2.setFont(prev);
    }

private:
    std::wstring text;
    tex::sptr<StbFont> font;
};

void ensureInit()
{
    static bool done = false;
    if (!done)
    {
        tex::LaTeX::init(MICROTEX_RES_DIR);
        done = true;
    }
}

} // namespace

} // namespace engine

// ---- MicroTeX platform factories (one backend per program, this is it) ----------

tex::Font* tex::Font::create(const std::string& file, float size)
{
    return new engine::StbFont(file, "", size);
}

tex::sptr<tex::Font> tex::Font::_create(const std::string& name, int style, float size)
{
    return std::make_shared<engine::StbFont>(engine::namedFontPath(name, style), name, size);
}

tex::sptr<tex::TextLayout> tex::TextLayout::create(const std::wstring& src,
                                                   const tex::sptr<tex::Font>& font)
{
    return std::make_shared<engine::StbTextLayout>(
        src, std::static_pointer_cast<engine::StbFont>(font));
}

namespace engine {

LatexImage::~LatexImage()
{
    if (textureId)
        glDeleteTextures(1, &textureId);
}

LatexImage::LatexImage(LatexImage&& o) noexcept
    : textureId(o.textureId), w(o.w), h(o.h), base(o.base), adv(o.adv),
      src(std::move(o.src)), err(std::move(o.err))
{
    o.textureId = 0;
}

LatexImage& LatexImage::operator=(LatexImage&& o) noexcept
{
    if (this != &o)
    {
        if (textureId)
            glDeleteTextures(1, &textureId);
        textureId = o.textureId;
        w = o.w;
        h = o.h;
        base = o.base;
        adv = o.adv;
        src = std::move(o.src);
        err = std::move(o.err);
        o.textureId = 0;
    }
    return *this;
}

bool LatexImage::set(const std::string& latex, float textSize, const glm::vec3& color)
{
    ensureInit();

    std::unique_ptr<tex::TeXRender> render;
    try
    {
        tex::color fg = tex::rgb(std::clamp(color.r, 0.0f, 1.0f),
                                 std::clamp(color.g, 0.0f, 1.0f),
                                 std::clamp(color.b, 0.0f, 1.0f));
        render.reset(tex::LaTeX::parse(tex::utf82wide(latex), 4096, textSize,
                                       textSize / 3.0f, fg));
    }
    catch (const std::exception& e)
    {
        err = e.what();
        return false;
    }
    if (!render)
    {
        err = "MicroTeX returned no render";
        return false;
    }

    const int pad = kPad;
    int iw = std::max(1, render->getWidth() + 2 * pad);
    int ih = std::max(1, render->getHeight() + 2 * pad);
    Canvas canvas(iw, ih);
    CanvasGraphics g2(canvas);
    render->draw(g2, pad, pad);

    // premultiplied float -> straight-alpha RGBA8 for ImGui's SRC_ALPHA blending
    std::vector<unsigned char> rgba((size_t)iw * ih * 4);
    for (size_t i = 0; i < (size_t)iw * ih; ++i)
    {
        const float* p = &canvas.px[i * 4];
        float a = p[3];
        for (int k = 0; k < 3; ++k)
            rgba[i * 4 + k] = (unsigned char)std::lround(
                std::clamp(a > 0.0f ? p[k] / a : 0.0f, 0.0f, 1.0f) * 255.0f);
        rgba[i * 4 + 3] = (unsigned char)std::lround(std::clamp(a, 0.0f, 1.0f) * 255.0f);
    }

    if (!textureId)
        glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, iw, ih, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 rgba.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    w = iw;
    h = ih;
    base = (float)(pad + render->getHeight() - render->getDepth());
    adv = (float)render->getWidth();
    src = latex;
    err.clear();
    return true;
}

void LatexImage::draw() const
{
    if (textureId)
        ImGui::Image((ImTextureID)(intptr_t)textureId, ImVec2((float)w, (float)h));
}

void LatexImage::drawAt(float x, float y, float alpha) const
{
    if (!textureId)
        return;
    ImU32 tint = IM_COL32(255, 255, 255, (int)(std::clamp(alpha, 0.0f, 1.0f) * 255.0f));
    ImGui::GetBackgroundDrawList()->AddImage((ImTextureID)(intptr_t)textureId,
                                             ImVec2(x, y), ImVec2(x + w, y + h),
                                             ImVec2(0, 0), ImVec2(1, 1), tint);
}

} // namespace engine
