# Aturan Pengembangan HVAC 2026 (Humanoid Virtual Athletics Challenge)

File ini mendefinisikan aturan ketat untuk kompetisi HVAC 2026 kategori Short Track yang wajib dipatuhi oleh agent AI (Antigravity) dalam project ini.

## Prinsip Dasar (Sim-to-Real)
1. **Tidak Boleh Curang dengan Fisika (No Physics Cheating):**
   - Dilarang mematikan deteksi tabrakan (collision) antar bagian robot maupun dengan lantai (`collisionDetection: true`).
   - Dilarang mengunci akar/root robot secara gaib di udara (`fix_root: false` pada `.cnoid`). Robot harus sepenuhnya menapak dan jatuh murni karena gravitasi.
   - Dilarang memberikan gaya (force/torque) secara ajaib melalui teleportasi `q_target` pada sendi virtual (`root_x`, `root_y`, `root_z`, `root_yaw`) untuk menjaga keseimbangan.

2. **Pergerakan Dinamis Murni:**
   - Semua pergerakan harus dihasilkan oleh torsi motor dari sendi-sendi nyata (pinggul, lutut, pergelangan kaki, lengan).
   - *Center of Mass* (CoM) dan ZMP (Zero Moment Point) harus dikontrol melalui algoritma berjalan/berlari, bukan ditopang oleh simulator.

3. **Inisialisasi Posisi Awal (Spawn):**
   - Robot harus di-*spawn* pada posisi ketinggian (Z) yang valid di mana telapak kaki tepat menyentuh permukaan lintasan (`Z=0` untuk field). Untuk robot ini, `initialRootPosition` biasanya adalah `[0, 0, 0.360]`.
   - `initialJointDisplacements` di `.cnoid` tidak boleh men-teleportasi sendi yang menyebabkan robot melayang tinggi atau menembus tanah.

4. **Kategori Short Track (20 Meter):**
   - Tujuan utama adalah mencapai garis akhir secepat mungkin. Evaluasi dihitung dari **elapsed simulation time** (bukan real-world time).
   - Lintasan memiliki geometri yang harus dihormati (batas lantai `Z = 0.0`). Jangan biarkan telapak kaki memotong geometri lintasan saat simulasi dimulai untuk menghindari tolakan pegas/ledakan fisika.

Setiap modifikasi kode (`.cpp`, `.cnoid`, `.urdf`) harus divalidasi terhadap prinsip-prinsip ini sebelum disarankan ke pengguna.
