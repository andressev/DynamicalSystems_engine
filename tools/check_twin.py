#!/usr/bin/env python3
"""Drift checker for the obsidian twin (decision 0004).

Cross-checks, in both directions:
  1. engine headers  <->  obsidian_twin/modules/<Name>.md
  2. sims/<name>/    <->  obsidian_twin/sims/<name-with-hyphens>.md
  3. every [[wikilink]] in the vault  ->  an existing note
  4. every `code:` path in note frontmatter  ->  an existing file
Also checks module notes declare a valid `layer:` in frontmatter.

Run from anywhere: python tools/check_twin.py   (exit 1 on any drift)
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VAULT = ROOT / "obsidian_twin"
ENGINE_HEADERS = ROOT / "engine" / "include" / "engine"
SIMS = ROOT / "sims"

LAYERS = {"core", "gfx", "math", "compute"}
WIKILINK = re.compile(r"\[\[([^\]|#]+)(?:[#|][^\]]*)?\]\]")


def frontmatter_code_paths(text):
    """Paths listed under `code:` in the note's frontmatter block."""
    m = re.match(r"\A---\n(.*?)\n---", text, re.DOTALL)
    if not m:
        return []
    block = re.search(r"^code:\s*\n((?:\s+-\s+.*\n?)+)", m.group(1) + "\n", re.MULTILINE)
    if not block:
        return []
    return re.findall(r"-\s+(\S.*?)\s*$", block.group(1), re.MULTILINE)


def vault_notes():
    return [p for p in VAULT.rglob("*.md") if ".obsidian" not in p.parts]


def main():
    problems = []

    # 1. headers <-> module notes
    headers = {p.stem for p in ENGINE_HEADERS.glob("*.h")}
    module_notes = {p.stem: p for p in (VAULT / "modules").glob("*.md")
                    if p.stem != "_template"}
    for h in sorted(headers - module_notes.keys()):
        problems.append(f"module '{h}' has no note (obsidian_twin/modules/{h}.md)")
    for n in sorted(module_notes.keys() - headers):
        problems.append(f"note modules/{n}.md has no header engine/include/engine/{n}.h")

    # module notes must declare a valid layer in frontmatter
    for name, path in sorted(module_notes.items()):
        text = path.read_text(encoding="utf-8-sig", errors="replace")
        m = re.search(r"^layer:\s*(\S+)", text, re.MULTILINE)
        if not m or m.group(1) not in LAYERS:
            problems.append(f"note modules/{name}.md missing valid 'layer:' "
                            f"frontmatter (one of {sorted(LAYERS)})")

    # 2. sim folders <-> sim notes (folder underscores -> note hyphens)
    sim_dirs = {p.name for p in SIMS.iterdir()
                if p.is_dir() and (p / "CMakeLists.txt").exists()}
    sim_notes = {p.stem for p in (VAULT / "sims").glob("*.md") if p.stem != "_template"}
    for d in sorted(sim_dirs):
        if d.replace("_", "-") not in sim_notes:
            problems.append(f"sim '{d}' has no note (obsidian_twin/sims/{d.replace('_', '-')}.md)")
    for n in sorted(sim_notes):
        if n.replace("-", "_") not in sim_dirs:
            problems.append(f"note sims/{n}.md has no sim folder sims/{n.replace('-', '_')}/")

    # 3. wikilinks resolve to existing notes; 4. `code:` frontmatter paths exist
    note_names = {p.stem for p in vault_notes()}
    for note in vault_notes():
        if note.stem == "_template":
            continue
        rel = note.relative_to(VAULT).as_posix()
        text = note.read_text(encoding="utf-8-sig", errors="replace")
        for target in WIKILINK.findall(text):
            if target.strip() not in note_names:
                problems.append(f"{rel}: broken wikilink [[{target.strip()}]]")
        for path in frontmatter_code_paths(text):
            if not (ROOT / path).exists():
                problems.append(f"{rel}: code path does not exist: {path}")

    if problems:
        print(f"obsidian twin drift — {len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
        return 1
    print(f"obsidian twin in sync: {len(headers)} modules, {len(sim_dirs)} sims, "
          f"all wikilinks resolve.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
