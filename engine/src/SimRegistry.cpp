#include <engine/SimRegistry.h>

namespace engine {

void SimRegistry::add(const std::string& name, SimFactory factory)
{
    simNames.push_back(name);
    factories.push_back(factory);
}

std::unique_ptr<Sim> SimRegistry::create(const std::string& name) const
{
    return createByIndex(indexOf(name));
}

std::unique_ptr<Sim> SimRegistry::createByIndex(int index) const
{
    if (index < 0 || index >= (int)factories.size())
        return nullptr;
    return factories[index]();
}

int SimRegistry::indexOf(const std::string& name) const
{
    for (int i = 0; i < (int)simNames.size(); ++i)
        if (simNames[i] == name)
            return i;
    return -1;
}

} // namespace engine
