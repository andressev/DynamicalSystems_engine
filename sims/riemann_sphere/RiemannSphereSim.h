#ifndef SIM_RIEMANN_SPHERE_H
#define SIM_RIEMANN_SPHERE_H

#include <engine/Sim.h>
#include <memory>

// Domain-coloring of a degree-7 complex polynomial on the Riemann sphere
// (stereographic projection). The sim class itself lives in the .cpp;
// apps only need this factory.
std::unique_ptr<engine::Sim> makeRiemannSphereSim();

#endif
