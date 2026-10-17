---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/GpuTimer.h
  - engine/src/GpuTimer.cpp
---
# GpuTimer

`engine/include/engine/GpuTimer.h`, `engine/src/GpuTimer.cpp`

## What it is
GPU time of a stretch of draw calls, from `GL_TIME_ELAPSED` queries — for profiling
readouts, since CPU timing of draw calls only measures *submission* (the GPU runs behind).
Owns its query objects (the "GL resources live in engine wrappers" rule).

## Dependencies
- Engine: none
- External: glad

## How to use it
1. Member: `engine::GpuTimer timer;` — queries are created on the first `begin()`, so a
   member constructed before the GL context is fine.
2. In `render`: `timer.begin();` … draw calls … `timer.end();`
3. `timer.lastMs()` — the newest *finished* measurement (a few frames old).
4. Used through [[TrajectoryRunner]]; reference sims: [[vector-field]],
   [[diagonal-linear]], [[lorenz-flow]].

## Notes
- A ring of 4 queries, read back only when `GL_QUERY_RESULT_AVAILABLE` — never stalls
  the pipeline; if all 4 are still in flight a frame's measurement is skipped.
- GL allows one `GL_TIME_ELAPSED` query active at a time: don't nest two timers.
- Non-copyable, movable; deletes its queries in the destructor (needs the context alive,
  which holds for sim members — [[App]] destroys the sim before the window).
