#!/usr/bin/env python3
"""Restore full collision geometry on every robot link from backup URDF."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
BACKUP = ROOT / "backup_before_floor_fd_20260524" / "robot_esr_v2_rev.urdf"
TARGET = ROOT / "urdf" / "robot_esr_v2_rev.urdf"

def main():
    if not BACKUP.is_file():
        raise SystemExit(f"Backup not found: {BACKUP}")
    shutil.copy2(BACKUP, TARGET)
    n = TARGET.read_text(encoding="utf-8").count("<collision>")
    print(f"Restored {TARGET} with {n} collision elements (all links)")

if __name__ == "__main__":
    main()
