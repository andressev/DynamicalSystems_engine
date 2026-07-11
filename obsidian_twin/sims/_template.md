---
type: sim
status: stable | wip
code:
  - sims/<name>/<Name>Sim.cpp
---
# <sim-name>

`sims/<name>/` — lib `sim_<name>`, standalone exe `<name>_standalone`.

One-sentence description of what this sim shows.

## What is being simulated
The phenomenon/object, in plain words: what a viewer is looking at and why it's
interesting.

## How it works
Rundown of the frame flow: what happens in `init` / `update` / `render` / `drawUI`,
which shaders are involved, where the interesting code lives.

## Modules attached
One line per module — what this sim uses it for:
- [[Shader]] — ...
- [[Mesh]] — ...
- [[Camera]] — mode + constraints set in `init`.

## The math
(Optional, as deep as needed.) What is being computed, key equations, where they live
(which shader / which C++ function). Shared concepts get their own note under `math/`
(from `math/_template.md`) and are linked from here instead of re-derived.

## Controls / uniforms
- UI widgets and what they drive.
- Uniforms and who sets them.

## Future
Planned extensions, links to [[roadmap]] milestones.
