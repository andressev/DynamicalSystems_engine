#ifndef ENGINE_SIM_REGISTRY_H
#define ENGINE_SIM_REGISTRY_H

#include <engine/Sim.h>
#include <memory>
#include <string>
#include <vector>

namespace engine {

using SimFactory = std::unique_ptr<Sim> (*)();

// Name -> factory map. Registration is explicit: each sim exposes a factory
// function in its header and the app's main() calls add() for the sims it wants.
class SimRegistry {
public:
    // thumbnailPath (optional) is an image shown on the sim-picker menu;
    // a missing/empty path falls back to a plain named tile.
    void add(const std::string& name, SimFactory factory,
             const std::string& thumbnailPath = "");

    std::unique_ptr<Sim> create(const std::string& name) const;
    std::unique_ptr<Sim> createByIndex(int index) const;
    int indexOf(const std::string& name) const; // -1 if unknown

    const std::vector<std::string>& names() const { return simNames; }
    const std::vector<std::string>& thumbnailPaths() const { return thumbPaths; }
    bool empty() const { return simNames.empty(); }

private:
    std::vector<std::string> simNames;
    std::vector<SimFactory> factories;
    std::vector<std::string> thumbPaths;
};

} // namespace engine

#endif
