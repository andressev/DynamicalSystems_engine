#ifndef ENGINE_GPU_TIMER_H
#define ENGINE_GPU_TIMER_H

namespace engine {

// GPU time of a stretch of draw calls (GL_TIME_ELAPSED queries), for profiling readouts.
// The GPU runs behind the CPU, so results are read a few frames late from a small ring of
// queries and never stall the pipeline: lastMs() is the newest finished measurement.
// Only one GpuTimer may be between begin() and end() at a time (a GL rule).
// Queries are created lazily on the first begin(), so a default-constructed member is
// fine before the GL context exists; non-copyable, movable.
class GpuTimer {
public:
    GpuTimer() = default;
    ~GpuTimer();
    GpuTimer(const GpuTimer&) = delete;
    GpuTimer& operator=(const GpuTimer&) = delete;
    GpuTimer(GpuTimer&& other) noexcept;
    GpuTimer& operator=(GpuTimer&& other) noexcept;

    void begin();
    void end();

    // Newest finished measurement in milliseconds (0 until one is available).
    double lastMs() const { return ms; }

private:
    static constexpr int kRing = 4;

    void release();

    unsigned int queries[kRing] = {};
    bool pending[kRing] = {};
    int next = 0;
    bool created = false;
    bool running = false;
    double ms = 0.0;
};

} // namespace engine

#endif
