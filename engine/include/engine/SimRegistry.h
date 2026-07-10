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
    void add(const std::string& name, SimFactory factory);

    std::unique_ptr<Sim> create(const std::string& name) const;
    std::unique_ptr<Sim> createByIndex(int index) const;
    int indexOf(const std::string& name) const; // -1 if unknown

    const std::vector<std::string>& names() const { return simNames; }
    bool empty() const { return simNames.empty(); }

private:
    std::vector<std::string> simNames;
    std::vector<SimFactory> factories;
};

} // namespace engine

#endif
