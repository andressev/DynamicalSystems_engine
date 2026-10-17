#include <engine/GpuTimer.h>

#include <glad/glad.h>

#include <utility>

namespace engine {

GpuTimer::~GpuTimer()
{
    release();
}

GpuTimer::GpuTimer(GpuTimer&& other) noexcept
{
    *this = std::move(other);
}

GpuTimer& GpuTimer::operator=(GpuTimer&& other) noexcept
{
    if (this != &other)
    {
        release();
        for (int i = 0; i < kRing; ++i)
        {
            queries[i] = other.queries[i];
            pending[i] = other.pending[i];
            other.queries[i] = 0;
            other.pending[i] = false;
        }
        next = other.next;
        created = other.created;
        running = other.running;
        ms = other.ms;
        other.created = false;
        other.running = false;
    }
    return *this;
}

void GpuTimer::release()
{
    if (created)
        glDeleteQueries(kRing, queries);
    created = false;
}

void GpuTimer::begin()
{
    if (running)
        return;
    if (!created)
    {
        glGenQueries(kRing, queries);
        created = true;
    }

    // collect every finished query without waiting; the newest one wins
    for (int k = 1; k <= kRing; ++k)
    {
        int i = (next + k) % kRing;   // oldest first
        if (!pending[i])
            continue;
        GLint available = 0;
        glGetQueryObjectiv(queries[i], GL_QUERY_RESULT_AVAILABLE, &available);
        if (!available)
            continue;
        GLuint64 ns = 0;
        glGetQueryObjectui64v(queries[i], GL_QUERY_RESULT, &ns);
        ms = (double)ns * 1e-6;
        pending[i] = false;
    }

    // every slot still in flight (GPU far behind): skip this measurement, don't stall
    if (pending[next])
        return;
    glBeginQuery(GL_TIME_ELAPSED, queries[next]);
    running = true;
}

void GpuTimer::end()
{
    if (!running)
        return;
    glEndQuery(GL_TIME_ELAPSED);
    pending[next] = true;
    next = (next + 1) % kRing;
    running = false;
}

} // namespace engine
