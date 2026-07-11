---
type: module
layer: core | gfx | math | compute
status: stable | wip
code:
  - engine/include/engine/<Name>.h
  - engine/src/<Name>.cpp
---
# <Name>

`engine/include/engine/<Name>.h`, `engine/src/<Name>.cpp`

## What it is
One paragraph: what this module does and why it exists. Link the decision or
roadmap milestone that motivated it if there is one.

## Dependencies
- Engine: [[OtherModule]] links, or "none".
- External: glad / glfw / glm / imgui / std-only.
- (Layer rule: a module may only depend on modules in the same or lower layer —
  math < gfx < compute, core is the skeleton. `math` modules must stay std-only.)

## How to use it
The attach recipe — what a sim adds to use this module:
1. Member declaration (the type, how it's constructed).
2. What happens in `init` / `update` / `render`.
3. Reference sim that uses it well: [[riemann-sphere]] etc.
Then the interface bullets: public methods/knobs and what they do.
Detaching = deleting the member and its call sites; nothing else should break.

## Notes
Invariants, gotchas, test status, planned extensions — anything the next user
would regret not knowing.
