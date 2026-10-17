#include <engine/FrameCopy.h>

#include <glad/glad.h>

namespace engine {

FrameCopy::~FrameCopy()
{
    release();
}

FrameCopy::FrameCopy(FrameCopy&& o) noexcept
    : tex(o.tex), fbo(o.fbo), vx(o.vx), vy(o.vy), w(o.w), h(o.h)
{
    o.tex = o.fbo = 0;
    o.w = o.h = 0;
}

FrameCopy& FrameCopy::operator=(FrameCopy&& o) noexcept
{
    if (this != &o)
    {
        release();
        tex = o.tex;
        fbo = o.fbo;
        vx = o.vx;
        vy = o.vy;
        w = o.w;
        h = o.h;
        o.tex = o.fbo = 0;
        o.w = o.h = 0;
    }
    return *this;
}

void FrameCopy::release()
{
    if (fbo)
        glDeleteFramebuffers(1, &fbo);
    if (tex)
        glDeleteTextures(1, &tex);
    fbo = tex = 0;
    w = h = 0;
}

void FrameCopy::capture()
{
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    if (vp[2] <= 0 || vp[3] <= 0)
        return;
    GLint readFbo = 0, drawFbo = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFbo);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFbo);

    if (!tex || vp[2] != w || vp[3] != h)
    {
        release();
        w = vp[2];
        h = vp[3];
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    }
    vx = vp[0];
    vy = vp[1];

    // read from what is being drawn into; a multisampled source is resolved by the blit
    glBindFramebuffer(GL_READ_FRAMEBUFFER, drawFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glBlitFramebuffer(vx, vy, vx + w, vy + h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, readFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFbo);
}

void FrameCopy::bind(int unit) const
{
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, tex);
    glActiveTexture(GL_TEXTURE0);
}

} // namespace engine
