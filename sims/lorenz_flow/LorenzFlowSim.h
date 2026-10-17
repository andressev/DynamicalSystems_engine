#ifndef LORENZ_FLOW_SIM_H
#define LORENZ_FLOW_SIM_H

#include <engine/Sim.h>
#include <memory>

std::unique_ptr<engine::Sim> makeLorenzFlowSim();

#endif
