#ifndef SIM_RIEMANN_SPHERE_H
#define SIM_RIEMANN_SPHERE_H

#include <engine/Sim.h>
#include <memory>

// Domain-coloring of a typed complex function f(z) on the Riemann sphere
// (stereographic projection), parsed and compiled to GLSL at runtime via
// engine::Expression. The sim class itself lives in the .cpp; apps only
// need this factory.
std::unique_ptr<engine::Sim> makeRiemannSphereSim();

#endif
