#ifndef SIM_LORENZ_MANIFOLD_H
#define SIM_LORENZ_MANIFOLD_H

#include <engine/Sim.h>
#include <memory>

std::unique_ptr<engine::Sim> makeLorenzManifoldSim();

#endif
