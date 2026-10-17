#ifndef ENGINE_FRAME_COPY_H
#define ENGINE_FRAME_COPY_H

namespace engine {

// A texture holding a copy of what has been drawn so far this frame, for effects that
// bend the scene behind them (a shockwave on glass, refraction). capture() blits the
// viewport of the framebuffer currently being drawn into — the window (multisampled:
// the blit resolves it) or App's offscreen screenshot target — into its own texture,
// resized to the viewport on demand, and restores the framebuffer bindings. Owns one
// texture + one framebuffer: non-copyable, movable; create after the GL context.
class FrameCopy {
public:
    FrameCopy() = default;
    ~FrameCopy();
    FrameCopy(const FrameCopy&) = delete;
    FrameCopy& operator=(const FrameCopy&) = delete;
    FrameCopy(FrameCopy&& other) noexcept;
    FrameCopy& operator=(FrameCopy&& other) noexcept;

    // Copy the current viewport of the bound draw framebuffer. Cheap enough per frame,
    // but call it only on frames that use the copy.
    void capture();

    // Bind the copy to texture unit `unit` (sample it with gl_FragCoord relative to the
    // viewport it was captured from: viewportX/Y/width/height).
    void bind(int unit) const;

    int viewportX() const { return vx; }
    int viewportY() const { return vy; }
    int width() const { return w; }
    int height() const { return h; }

private:
    void release();

    unsigned int tex = 0;
    unsigned int fbo = 0;
    int vx = 0, vy = 0, w = 0, h = 0;
};

} // namespace engine

#endif
