#ifndef ENGINE_LEVEL_SET_SURFACE_H
#define ENGINE_LEVEL_SET_SURFACE_H

#include <map>
#include <string>
#include <vector>

namespace engine {

// A 2-D manifold grown by geodesic level sets, stored band by band so it can be saved
// once and grown again later without solving anything: band k is the strip of
// triangles between level set k and level set k + 1. Every vertex keeps where it
// grew from (its start on the previous level set; a vertex on that level set grew
// from itself), so a band can be drawn partly grown, from -> pos. Math layer: std-only.
class LevelSetSurface {
public:
    struct Band {
        double eta = 0.0;          // geodesic distance of its outer level set from the edge it started at
        std::vector<float> pos;    // triangles, xyz per vertex
        std::vector<float> from;   // same layout: where each vertex grew from
    };

    std::map<std::string, double> tags;   // what it was grown for (parameters, radius, ...)
    std::vector<Band> bands;

    // Binary file: tags, then every band. False if it cannot be written / read (a
    // failed load leaves the surface empty).
    bool save(const std::string& path) const;
    bool load(const std::string& path);

    double tag(const std::string& key, double fallback = 0.0) const;
    size_t vertexCount() const;
    bool empty() const { return bands.empty(); }
};

} // namespace engine

#endif
