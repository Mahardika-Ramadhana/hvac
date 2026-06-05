#!/usr/bin/env python3
"""Generate BodyMotion .seq files for robot_esr_v2 (34 joints).

Joint index map (verified from URDF):
   0  right_knee_yaw_2    (R hip yaw)
   1  right_knee_roll_2   (R hip roll)
   2  right_knee_pitch_2  (R hip pitch)
   3  right_knee_pitch_4  (R knee)
   4  right_knee_pitch_6  (R ankle pitch)
   5  right_knee_roll_6   (R ankle roll)
   6  left_knee_yaw_1     (L hip yaw)
   7  left_knee_roll_1    (L hip roll)
   8  left_knee_pitch_1   (L hip pitch)
   9  left_knee_pitch_3   (L knee)
  10  left_knee_pitch_5   (L ankle pitch)
  11  left_knee_roll_5    (L ankle roll)
  12  body_pitch
  13  body_yaw
  14  right_arm_pitch_2   (R shoulder pitch)
  15  right_arm_roll_2    (R shoulder roll)
  16  right_arm_yaw_2     (R shoulder yaw)
  17  right_arm_pitch_4   (R elbow)
  18  right_arm_pitch_6   (R wrist pitch)
  19  right_thumb_2
  20  right_thumb_4
  21  right_finger_2
  22  right_finger_4
  23  left_arm_pitch_1    (L shoulder pitch)
  24  left_arm_roll_1     (L shoulder roll)
  25  left_arm_yaw_1      (L shoulder yaw)
  26  left_arm_pitch_3    (L elbow)
  27  left_arm_pitch_5    (L wrist pitch)
  28  left_thumb_1
  29  left_thumb_3
  30  left_finger_1
  31  left_finger_3
  32  head_yaw
  33  head_pitch

Output: BodyMotion file with BOTH JointDisplacement (34 parts) and
LinkPosition (1 part – root link), so Choreonoid's BodyMotionEngine
will translate the body forward/backward during playback (no simulator
needed, no falling possible).
"""

import math
from pathlib import Path

NUM_JOINTS = 38     # 4 virtual root joints + 34 robot joints
FRAME_RATE = 500

# ── joint indices ────────────────────────────────────────────────────
# Virtual root joints (added in URDF to enable world-frame translation
# in Kinematics simulation – same trick as the previous working robot)
ROOT_X, ROOT_Y, ROOT_Z, ROOT_YAW = 0, 1, 2, 3

# Real robot joints (shifted +4 from before)
R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_P, R_ANKLE_R = 4, 5, 6, 7, 8, 9
L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_P, L_ANKLE_R = 10, 11, 12, 13, 14, 15
BODY_PITCH, BODY_YAW = 16, 17
R_SHO_P, R_SHO_R, R_SHO_Y, R_ELBOW, R_WRIST = 18, 19, 20, 21, 22
L_SHO_P, L_SHO_R, L_SHO_Y, L_ELBOW, L_WRIST = 27, 28, 29, 30, 31
HEAD_YAW, HEAD_PITCH = 36, 37

# ── stand pose ───────────────────────────────────────────────────────
HIP_PITCH_BASE = -0.04
KNEE_BASE      = +0.04
BODY_PITCH_STAND = +0.05
SHOULDER_BASE  = +0.10   # arms slightly forward at rest
ELBOW_BASE     = -0.25   # gentle elbow flex

ROOT_HEIGHT = 0.81       # initial root Z

def stand_pose():
    q = [0.0] * NUM_JOINTS
    # ── virtual root: body lifted to standing height ───────────────
    q[ROOT_X]   = 0.0
    q[ROOT_Y]   = 0.0
    q[ROOT_Z]   = ROOT_HEIGHT
    q[ROOT_YAW] = 0.0
    # legs
    q[R_HIP_PITCH] = +HIP_PITCH_BASE
    q[R_KNEE]      = +KNEE_BASE
    q[R_ANKLE_P]   = q[R_KNEE] - q[R_HIP_PITCH]
    q[L_HIP_PITCH] = -HIP_PITCH_BASE
    q[L_KNEE]      = -KNEE_BASE
    q[L_ANKLE_P]   = q[L_KNEE] - q[L_HIP_PITCH]
    # body
    q[BODY_PITCH]  = BODY_PITCH_STAND
    # arms at rest – arms hang with slight forward pitch
    q[R_SHO_P]     = +SHOULDER_BASE
    q[L_SHO_P]     = +SHOULDER_BASE
    q[R_ELBOW]     = ELBOW_BASE
    q[L_ELBOW]     = ELBOW_BASE
    return q


# ── helpers ──────────────────────────────────────────────────────────
def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a + (b - a) * t


# ── core gait frame ──────────────────────────────────────────────────
def gait_frame(t, T, mode='forward'):
    """One frame of biped walking at time t within cycle of period T.

    mode: 'place'    – walk in place
          'forward'  – walk forward  (hip pitch swing +)
          'backward' – walk backward (hip pitch swing -, arms reversed)
    """
    base = stand_pose()
    q = base[:]

    omega = 2.0 * math.pi * (t % T) / T
    s = math.sin(omega)
    c = math.cos(omega)

    # ── amplitudes – BIGGER for visible motion ─────────────────────
    A_hip   = {'place': 0.00, 'forward': 0.30, 'backward': -0.30}.get(mode, 0.0)
    A_knee  = 0.55        # bigger knee lift  (was 0.35)
    A_roll  = 0.12        # lateral hip roll
    A_arm   = 0.45        # shoulder pitch swing  (was 0.25)
    A_sho_r = 0.05        # shoulder roll sway
    A_elbow = 0.25        # elbow flex during swing
    A_wrist = 0.12        # wrist flex
    A_body_yaw = 0.05     # body yaw oscillation
    A_body_pitch_osc = 0.03   # body pitch bob
    A_head_yaw = 0.04     # subtle head turn
    A_head_pitch = 0.03   # subtle head nod

    # ── hip pitch (same q both legs → anti-phase in world) ─────────
    q[R_HIP_PITCH] = +HIP_PITCH_BASE + A_hip * s
    q[L_HIP_PITCH] = -HIP_PITCH_BASE + A_hip * s

    # ── knee lift only on swing leg ────────────────────────────────
    right_swing = max(0.0, s)
    left_swing  = max(0.0, -s)
    q[R_KNEE] = +KNEE_BASE + A_knee * right_swing
    q[L_KNEE] = -KNEE_BASE - A_knee * left_swing

    # ── ankle: keep foot ~flat ─────────────────────────────────────
    q[R_ANKLE_P] = q[R_KNEE] - q[R_HIP_PITCH]
    q[L_ANKLE_P] = q[L_KNEE] - q[L_HIP_PITCH]

    # ── hip yaw subtle oscillation (foot orientation) ──────────────
    q[R_HIP_YAW] = +0.02 * s
    q[L_HIP_YAW] = +0.02 * s

    # ── hip roll for lateral COM shift ─────────────────────────────
    q[R_HIP_ROLL] = +A_roll * s
    q[L_HIP_ROLL] = +A_roll * s

    # ── ankle roll (compensate for hip roll) ───────────────────────
    q[R_ANKLE_R] = -A_roll * 0.5 * s
    q[L_ANKLE_R] = -A_roll * 0.5 * s

    # ── arms anti-phase via axis mirror, flipped for backward ──────
    arm_sign = +1.0 if mode == 'backward' else -1.0
    q[R_SHO_P] = +SHOULDER_BASE + arm_sign * A_arm * s
    q[L_SHO_P] = +SHOULDER_BASE + arm_sign * A_arm * s
    q[R_SHO_R] = +A_sho_r * s
    q[L_SHO_R] = +A_sho_r * s
    q[R_ELBOW] = ELBOW_BASE - A_elbow * abs(s)
    q[L_ELBOW] = ELBOW_BASE - A_elbow * abs(s)
    q[R_WRIST] = +A_wrist * s
    q[L_WRIST] = +A_wrist * s

    # ── body ───────────────────────────────────────────────────────
    q[BODY_PITCH] = BODY_PITCH_STAND + A_body_pitch_osc * c
    q[BODY_YAW]   = A_body_yaw * s

    # ── head ───────────────────────────────────────────────────────
    q[HEAD_YAW]   = -A_head_yaw * s          # head looks opposite to body yaw
    q[HEAD_PITCH] = +A_head_pitch * c        # subtle nod with body bob

    # ── finger small clench ────────────────────────────────────────
    finger_q = 0.05 * abs(s)
    q[19] = finger_q; q[20] = finger_q   # right thumb
    q[21] = finger_q; q[22] = finger_q   # right fingers
    q[28] = finger_q; q[29] = finger_q   # left thumb
    q[30] = finger_q; q[31] = finger_q   # left fingers

    return q


# ── root translation trajectory ──────────────────────────────────────
WALK_SPEED = 0.25   # m/s

# Robot's "forward" direction in world frame – the visual orientation
# of the URDF mesh makes +Y appear as BACKWARD, so the schedule below
# is negated. (User reported maju/mundur swapped; this flips it.)
FORWARD_SIGN = -1.0

def root_translation_at(t):
    """Return (x, y, z, yaw) at global time t. Yaw is always 0 in demo."""
    if t <= 5.0:
        y = 0.0
    elif t <= 15.0:
        y = (t - 5.0) * WALK_SPEED            # 0 → 2.5 m forward
    elif t <= 23.0:
        y = 2.5 - (t - 15.0) * WALK_SPEED     # 2.5 → 0.5 m (backward)
    else:
        y = 0.5
    bob = 0.004 * math.cos(2.0 * math.pi * t / 1.4)   # tiny up/down bob
    return (0.0, FORWARD_SIGN * y, ROOT_HEIGHT + bob, 0.0)


# ── segment helpers ──────────────────────────────────────────────────
def apply_root(q, t):
    """Overwrite the 4 virtual root joints based on the global timeline."""
    rx, ry, rz, ryaw = root_translation_at(t)
    q[ROOT_X]   = rx
    q[ROOT_Y]   = ry
    q[ROOT_Z]   = rz
    q[ROOT_YAW] = ryaw
    return q


def hold_pose(pose, duration_s, t0=0.0):
    """Hold a static pose for duration_s seconds. Updates root trajectory
    so that base_link still follows the global root schedule."""
    n = max(1, int(round(duration_s * FRAME_RATE)))
    frames = []
    for i in range(n):
        q = pose[:]
        apply_root(q, t0 + i / FRAME_RATE)
        frames.append(q)
    return frames


def gait_segment(duration_s, T, mode, t0=0.0,
                 fade_in=True, fade_out=True, fade_s=0.5):
    """Generate a gait segment, optionally fading in/out from/to stand pose.
    t0 is the start time of this segment within the global timeline,
    used to compute the virtual root trajectory."""
    n = max(1, int(round(duration_s * FRAME_RATE)))
    base = stand_pose()
    fade_n = max(1, int(fade_s * FRAME_RATE))
    frames = []
    for i in range(n):
        t_local  = i / FRAME_RATE
        t_global = t0 + t_local
        q_walk = gait_frame(t_local, T, mode)
        if fade_in and i < fade_n:
            a = smoothstep(i / fade_n)
            q = [lerp(b, w, a) for b, w in zip(base, q_walk)]
        elif fade_out and i >= n - fade_n:
            a = smoothstep((n - 1 - i) / fade_n)
            q = [lerp(b, w, a) for b, w in zip(base, q_walk)]
        else:
            q = q_walk
        apply_root(q, t_global)
        frames.append(q)
    return frames


# ── full demo motion: 5s in-place + 10s forward + 8s backward ───────
def generate_full_motion():
    base = stand_pose()
    frames = []
    # 0 – 5 s : walk IN PLACE (fade in from stand)
    frames += gait_segment(5.0,  T=1.4, mode='place',    t0=0.0,
                           fade_in=True,  fade_out=False, fade_s=0.6)
    # 5 – 15 s : walk FORWARD
    frames += gait_segment(10.0, T=1.4, mode='forward',  t0=5.0,
                           fade_in=False, fade_out=False, fade_s=0.6)
    # 15 – 23 s : walk BACKWARD (fade out)
    frames += gait_segment(8.0,  T=1.4, mode='backward', t0=15.0,
                           fade_in=False, fade_out=True,  fade_s=0.6)
    # 23 – 25 s : settle in stand pose
    frames += hold_pose(base, 2.0, t0=23.0)
    return frames


# ── seq writer (joint-only) ──────────────────────────────────────────
def fmt(values):
    return "[ " + ", ".join(
        "0" if abs(v) < 1e-9 else f"{v:.6g}" for v in values
    ) + " ]"


def fmt_se3(se3):
    return "[ " + ", ".join(
        "0" if abs(v) < 1e-9 else f"{v:.6g}" for v in se3
    ) + " ]"


def write_seq(path, frames):
    """Write joint-only BodyMotion (.seq)."""
    n = len(frames)
    lines = [
        "type: CompositeSeq",
        "content: BodyMotion",
        "formatVersion: 2",
        f"frameRate: {FRAME_RATE}",
        f"numFrames: {n}",
        "components:",
        "  -",
        "    type: MultiValueSeq",
        "    content: JointDisplacement",
        f"    numParts: {NUM_JOINTS}",
        "    frames:",
    ]
    for f in frames:
        lines.append(f"      - {fmt(f)}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {path} ({n} frames = {n/FRAME_RATE:.1f} s @ {FRAME_RATE} fps)")


def main():
    motion_dir = Path(__file__).resolve().parent.parent / "motion"
    # Static stand pose held for 5 s (just for testing the rest position)
    write_seq(motion_dir / "stand.seq",        hold_pose(stand_pose(), 5.0))
    # Walk in place 8 s (loopable)
    write_seq(motion_dir / "walk_in_place.seq",
              gait_segment(8.0, 1.4, 'place', t0=0.0))

    # Main demo motion: 5 s in place + 10 s forward + 8 s backward + 2 s settle.
    # Root translation lives in joints 0–3 (root_x, root_y, root_z, root_yaw)
    # so AISTSimulator in Kinematics mode will reproduce it via q_target.
    demo = generate_full_motion()
    write_seq(motion_dir / "walk_demo.seq",    demo)
    write_seq(motion_dir / "walk_forward.seq", demo)


if __name__ == "__main__":
    main()
