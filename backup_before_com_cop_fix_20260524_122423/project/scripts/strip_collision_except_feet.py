#!/usr/bin/env python3
"""Keep collision geometry only on foot links for stable floor contact."""
import re
import sys
from pathlib import Path

FOOT_LINKS = {"right_knee_roll_6", "left_knee_roll_5"}

def strip_collisions(text: str) -> str:
    # Split into link blocks and process each
    out = []
    pos = 0
    link_pat = re.compile(r'(<link\s+[^>]*name="([^"]+)"[^>]*>)(.*?)(</link>)', re.DOTALL)
    for m in link_pat.finditer(text):
        out.append(text[pos:m.start()])
        name = m.group(2)
        inner = m.group(3)
        if name not in FOOT_LINKS:
            inner = re.sub(r'\s*<collision>.*?</collision>\s*', '\n', inner, flags=re.DOTALL)
        out.append(m.group(1) + inner + m.group(4))
        pos = m.end()
    out.append(text[pos:])
    return "".join(out)

def main():
    urdf = Path(__file__).resolve().parents[2] / "urdf" / "robot_esr_v2.urdf"
    text = urdf.read_text(encoding="utf-8")
    before = text.count("<collision>")
    new_text = strip_collisions(text)
    after = new_text.count("<collision>")
    urdf.write_text(new_text, encoding="utf-8")
    print(f"Updated {urdf}")
    print(f"Collision elements: {before} -> {after} (feet only)")

if __name__ == "__main__":
    main()
