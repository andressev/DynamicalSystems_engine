#ifndef LORENZ_EQUILIBRIUM_SIM_H
#define LORENZ_EQUILIBRIUM_SIM_H

#include <engine/Sim.h>
#include <memory>

std::unique_ptr<engine::Sim> makeLorenzEquilibriumSim();

#endif
