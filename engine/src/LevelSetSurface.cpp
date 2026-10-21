#include <engine/LevelSetSurface.h>

#include <cstdint>
#include <cstring>
#include <fstream>

namespace engine {

namespace {

constexpr char kMagic[4] = {'L', 'S', 'S', '1'};

template <typename T>
void put(std::ofstream& out, const T& v)
{
    out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool get(std::ifstream& in, T& v)
{
    return (bool)in.read(reinterpret_cast<char*>(&v), sizeof(T));
}

} // namespace

bool LevelSetSurface::save(const std::string& path) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out)
        return false;
    out.write(kMagic, 4);
    put(out, (uint32_t)tags.size());
    for (const auto& [key, value] : tags)
    {
        put(out, (uint32_t)key.size());
        out.write(key.data(), (std::streamsize)key.size());
        put(out, value);
    }
    put(out, (uint32_t)bands.size());
    for (const Band& b : bands)
    {
        const uint32_t n = (uint32_t)(b.pos.size() / 3);
        put(out, b.eta);
        put(out, n);
        out.write(reinterpret_cast<const char*>(b.pos.data()), (std::streamsize)(n * 3 * sizeof(float)));
        out.write(reinterpret_cast<const char*>(b.from.data()), (std::streamsize)(n * 3 * sizeof(float)));
    }
    return (bool)out;
}

bool LevelSetSurface::load(const std::string& path)
{
    tags.clear();
    bands.clear();
    std::ifstream in(path, std::ios::binary);
    char magic[4];
    if (!in || !in.read(magic, 4) || std::memcmp(magic, kMagic, 4) != 0)
        return false;
    auto fail = [&] {
        tags.clear();
        bands.clear();
        return false;
    };
    uint32_t count = 0;
    if (!get(in, count) || count > 1024)
        return fail();
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t len = 0;
        if (!get(in, len) || len > 256)
            return fail();
        std::string key(len, '\0');
        double value = 0.0;
        if (!in.read(key.data(), len) || !get(in, value))
            return fail();
        tags[key] = value;
    }
    if (!get(in, count) || count > (1u << 20))
        return fail();
    bands.resize(count);
    for (Band& b : bands)
    {
        uint32_t n = 0;
        if (!get(in, b.eta) || !get(in, n) || n > (1u << 26))
            return fail();
        b.pos.resize((size_t)n * 3);
        b.from.resize((size_t)n * 3);
        if (!in.read(reinterpret_cast<char*>(b.pos.data()), (std::streamsize)(n * 3 * sizeof(float))) ||
            !in.read(reinterpret_cast<char*>(b.from.data()), (std::streamsize)(n * 3 * sizeof(float))))
            return fail();
    }
    return true;
}

double LevelSetSurface::tag(const std::string& key, double fallback) const
{
    auto it = tags.find(key);
    return it == tags.end() ? fallback : it->second;
}

size_t LevelSetSurface::vertexCount() const
{
    size_t n = 0;
    for (const Band& b : bands)
        n += b.pos.size() / 3;
    return n;
}

} // namespace engine
