# Robot Besar — Choreonoid Simulation

## Quick start

```bash
bash project/run_cobagerakin.sh
```

1. Pilih **AISTSimulator-Position** (lebih stabil) atau **AISTSimulator** (torque PD).
2. Klik **Start simulation** (SimulationBar).
3. Robot memakai forward dynamics + pola gerak dari `project/motion/`.

## Mode gerak

Ubah properti **controllerOptions** pada simulator yang aktif:

| Nilai | Motion |
|-------|--------|
| `stand` | Berdiri (pose tetap) |
| `in_place` | Jalan di tempat |
| `forward` | Jalan maju |
| `position` | Jalan dengan kontrol posisi joint (disarankan untuk uji awal) |

**Tips stabilitas:** gunakan `AISTSimulator-Position` + `position` atau `in_place` sebelum mode torque (`AISTSimulator` + `in_place`). Sesuaikan tinggi awal `rootPosition` Z di properti robot jika kaki tidak menyentuh lantai.

Untuk hanya menahan pose awal, ganti controller pada `robot_esr_v2` menjadi `RobotBesarMinimumController` dan set `controllerOptions` ke `stand`.

## Build ulang controller

```bash
cd project/controller
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Generate ulang motion

```bash
python3 project/scripts/generate_walk_motion.py
```
