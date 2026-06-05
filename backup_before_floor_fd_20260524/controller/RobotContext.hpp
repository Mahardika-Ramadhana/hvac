#ifndef ROBOT_ESR_V2_ROBOT_CONTEXT_HPP
#define ROBOT_ESR_V2_ROBOT_CONTEXT_HPP

// Standard C++
#include <cstddef>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

// Choreonoid
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Link>

using namespace cnoid;

// ============================================================
//  RobotContext – shared abstraction over robot_esr_v2 (38 DOF)
//
//  Joint layout assumed (after URDF injection of 4 virtual root
//  joints; see urdf/robot_esr_v2.urdf):
//      0 : root_x_joint   (prismatic X)
//      1 : root_y_joint   (prismatic Y)
//      2 : root_z_joint   (prismatic Z)
//      3 : root_yaw_joint (revolute Z)
//      4 : right_knee_yaw_2    -- right leg yaw at hip
//      5 : right_knee_roll_2   -- right hip roll
//      6 : right_knee_pitch_2  -- right hip pitch
//      7 : right_knee_pitch_4  -- right knee pitch
//      8 : right_knee_pitch_6  -- right ankle pitch
//      9 : right_knee_roll_6   -- right ankle roll
//     10 : left_knee_yaw_1     -- left leg yaw
//     11 : left_knee_roll_1    -- left hip roll
//     12 : left_knee_pitch_1   -- left hip pitch
//     13 : left_knee_pitch_3   -- left knee pitch
//     14 : left_knee_pitch_5   -- left ankle pitch
//     15 : left_knee_roll_5    -- left ankle roll
//     16 : body_pitch
//     17 : body_yaw
//     18-26 : right arm + hand
//     27-35 : left arm + hand
//     36 : head_yaw
//     37 : head_pitch
// ============================================================
struct RobotContext
{
    Body*  body = nullptr;
    double dt   = 0.0;

    std::vector<Link*> joints;   // every joint except the virtual root_z
    std::vector<double> q0;      // neutral pose (per jointId)

    // ── virtual root joints ─────────────────────────────────
    Link* root_x   = nullptr;
    Link* root_y   = nullptr;
    Link* root_z   = nullptr;
    Link* root_yaw = nullptr;

    // ── leg yaw / roll at hip ──────────────────────────────
    Link* l_leg_yaw  = nullptr;
    Link* r_leg_yaw  = nullptr;
    Link* l_hip_roll = nullptr;
    Link* r_hip_roll = nullptr;

    // ── pitch chain (hip → knee → ankle) ────────────────────
    Link* l_hip   = nullptr;
    Link* l_knee  = nullptr;
    Link* l_ankle = nullptr;

    Link* r_hip   = nullptr;
    Link* r_knee  = nullptr;
    Link* r_ankle = nullptr;

    Link* l_ankle_roll = nullptr;
    Link* r_ankle_roll = nullptr;

    // ── arms ────────────────────────────────────────────────
    Link* l_arm = nullptr;
    Link* r_arm = nullptr;

    // ── torso ───────────────────────────────────────────────
    Link* body_pitch = nullptr;
    Link* body_yaw   = nullptr;

    // ── head ────────────────────────────────────────────────
    Link* head_yaw   = nullptr;
    Link* head_pitch = nullptr;

    bool   initialize(SimpleControllerIO* io);

    double q0of(Link* j) const;

    void   setTarget(Link* j, double q);
    void   resetAllTargets();

    void   setRoot(double x, double y, double z, double yaw);
    void   keepYawNeutral();

    // ── helper math ────────────────────────────────────────
    static double clamp01(double x);
    static double smoothstep(double x);
    static double blend(double from, double to, double s);
    static double forwardRootY(double moveTime,
                               double speed,
                               double rootRampDur,
                               double duration);
};

#endif
