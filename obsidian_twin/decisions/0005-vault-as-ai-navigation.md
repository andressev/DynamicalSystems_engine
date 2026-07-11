# 0005 — The vault carries *why*, *where*, and *now* — never *what the code says*

**Date:** 2026-07-11 · **Status:** accepted

## Context
The obsidian twin should make the project understandable fast — for humans and for AI
sessions that start cold. An AI reads code quickly, so notes that describe code are
worthless (and rot); the expensive questions are *why is it built this way*, *where
does concept X live*, and *what's the current state*.

## Decision
1. **Routing over reading**: [[Home]] opens with a task → notes table ("if you're doing
   X, read Y first"), and CLAUDE.md instructs following it before non-trivial work. The
   vault is a dispatcher, not a library to read cover-to-cover.
2. **`math/` folder**: shared domain concepts (projection, coloring, integrators...)
   are first-class notes — idea / equations / where implemented / pitfalls — linked
   from sims instead of re-derived in each. One concept note serves every sim using it.
3. **Machine-verifiable pointers**: module/sim/math notes list their source files under
   `code:` in frontmatter; `tools/check_twin.py` verifies the paths exist. A green
   check means the vault's pointers into code are current and can be trusted.
4. **`## Now` section atop [[roadmap]]** (≤5 lines): current milestone, mid-flight
   work, next step — the only volatile state note, kept tiny so it actually gets
   updated. Git shows what happened; Now says what's *in progress*.
5. Rejected: per-file exhaustive docs and code snippets in notes (rot), a changelog
   note (git does it better), deeper folder hierarchy (navigation cost).

## Consequences
- A cold session reads Home's routing table + roadmap's Now and knows where to go.
- Function names in prose remain unchecked (only file paths are verified) — acceptable;
  revisit if they start rotting.
