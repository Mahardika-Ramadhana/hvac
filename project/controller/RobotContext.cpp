#include "RobotContext.hpp"
#include <cmath>

bool RobotContext::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt   = io->timeStep();

    joints.clear();
    q0.assign(body->numJoints(), 0.0);

    std::cout << "=== RobotContext (robot_esr_v2) initialize ===" << std::endl;
    std::cout << "Robot name   : " << body->name() << std::endl;
    std::cout << "Jumlah joint : " << body->numJoints() << std::endl;

    for(int i = 0; i < body->numJoints(); ++i){
        Link* j = body->joint(i);

        if(j->jointId() >= 0 && j->jointId() < (int)q0.size()){
            q0[j->jointId()] = 0.0;
        }

        // NOTE: actuation mode + io enable are now configured by the
        // higher-level controller AFTER it knows whether we're running
        // in kinematics or PID-torque mode.  Doing it here once led to
        // an inconsistent mix when the mode was later overridden.

        // root_z held separately by setRoot
        if(j->name() != "root_z_joint" && j->name() != "root_z_link"){
            joints.push_back(j);
        }

        std::cout << i << " : " << j->name()
                  << " | q awal = " << j->q() << std::endl;
    }

    // ── lookup virtual root joints ─────────────────────────
    root_x   = body->joint("root_x_joint");
    root_y   = body->joint("root_y_joint");
    root_z   = body->joint("root_z_joint");
    root_yaw = body->joint("root_yaw_joint");

    // ── pitch chain ───────────────────────────────────────
    r_hip   = body->joint("right_knee_pitch_2");
    r_knee  = body->joint("right_knee_pitch_4");
    r_ankle = body->joint("right_knee_pitch_6");

    l_hip   = body->joint("left_knee_pitch_1");
    l_knee  = body->joint("left_knee_pitch_3");
    l_ankle = body->joint("left_knee_pitch_5");

    // ── yaw / roll at hip ─────────────────────────────────
    r_leg_yaw  = body->joint("right_knee_yaw_2");
    l_leg_yaw  = body->joint("left_knee_yaw_1");
    r_hip_roll = body->joint("right_knee_roll_2");
    l_hip_roll = body->joint("left_knee_roll_1");

    // ── ankle roll ────────────────────────────────────────
    r_ankle_roll = body->joint("right_knee_roll_6");
    l_ankle_roll = body->joint("left_knee_roll_5");

    // ── arms (pitch1 = shoulder front/back swing) ─────────
    r_arm = body->joint("right_arm_pitch_2");
    l_arm = body->joint("left_arm_pitch_1");

    // ── torso ─────────────────────────────────────────────
    body_pitch = body->joint("body_pitch");
    body_yaw   = body->joint("body_yaw");

    // ── head ──────────────────────────────────────────────
    head_yaw   = body->joint("head_yaw");
    head_pitch = body->joint("head_pitch");

    // ── neutral stand pose ────────────────────────────────
    //  Tiny crouch for stability when grounded later.
    //  Sign convention: q0 added directly (we use it via q0of()).
    //  Knee bend (positive on right_knee_pitch_4) folds the knee
    //  the way a human knee folds (lower leg goes back).  We add
    //  matching hip+ankle bias so foot remains under hip.
    const double standKneeFlex  = 0.10;
    const double standHipFlex   = 0.05;
    const double standAnkleFlex = 0.05;

    auto bias = [&](Link* j, double v){
        if(j && j->jointId() >= 0) q0[j->jointId()] += v;
    };

    bias(r_knee,  standKneeFlex);
    bias(l_knee,  standKneeFlex);
    bias(r_hip,   standHipFlex);
    bias(l_hip,   standHipFlex);
    bias(r_ankle, standAnkleFlex);
    bias(l_ankle, standAnkleFlex);

    // Initial floating-base height: keep the URDF default (the
    // .cnoid initialJointDisplacements puts root_z at 0.81 m).
    // We do not overwrite q0 for root joints here.

    std::cout << "=== Joint mapping ===" << std::endl;
    auto dump = [](const char* tag, Link* j){
        std::cout << tag << ": " << (j ? j->name() : "NULL") << std::endl;
    };
    dump("root_x   ", root_x);
    dump("root_y   ", root_y);
    dump("root_z   ", root_z);
    dump("root_yaw ", root_yaw);
    dump("r_hip    ", r_hip);
    dump("r_knee   ", r_knee);
    dump("r_ankle  ", r_ankle);
    dump("l_hip    ", l_hip);
    dump("l_knee   ", l_knee);
    dump("l_ankle  ", l_ankle);
    dump("r_arm    ", r_arm);
    dump("l_arm    ", l_arm);
    dump("body_p   ", body_pitch);
    dump("body_yaw ", body_yaw);

    return true;
}

double RobotContext::q0of(Link* j) const
{
    if(j && j->jointId() >= 0 && j->jointId() < (int)q0.size()){
        return q0[j->jointId()];
    }
    return 0.0;
}

void RobotContext::setTarget(Link* j, double q)
{
    if(j){
        j->q_target() = q;
    }
}

void RobotContext::resetAllTargets()
{
    for(Link* j : joints){
        j->q_target() = q0of(j);
    }
    keepYawNeutral();
    if(body_pitch){
        body_pitch->q_target() = q0of(body_pitch);
    }
}

void RobotContext::setRoot(double x, double y, double z, double yaw,
                           bool lockHeight)
{
    if(root_x)   root_x->q_target()   = x;
    if(root_y)   root_y->q_target()   = y;
    if(root_z && lockHeight){
        root_z->q_target() = STAND_ROOT_Z;
    }
    if(root_yaw) root_yaw->q_target() = yaw;
}

void RobotContext::setRootWithZOffset(double x, double y, double zOffset, double yaw)
{
    if(root_x)   root_x->q_target()   = x;
    if(root_y)   root_y->q_target()   = y;
    if(root_z)   root_z->q_target()   = STAND_ROOT_Z + zOffset;
    if(root_yaw) root_yaw->q_target() = yaw;
}

void RobotContext::keepYawNeutral()
{
    if(l_leg_yaw) l_leg_yaw->q_target() = q0of(l_leg_yaw);
    if(r_leg_yaw) r_leg_yaw->q_target() = q0of(r_leg_yaw);
    if(body_yaw)  body_yaw->q_target()  = q0of(body_yaw);
}

void RobotContext::applyStandPose(bool setActualQ,
                                  double keepX, double keepY, double keepYaw)
{
    auto set = [&](Link* j, double v){
        if(!j) return;
        if(setActualQ) j->q() = v;
        j->q_target() = v;
    };

    set(root_x,   keepX);
    set(root_y,   keepY);
    set(root_z,   STAND_ROOT_Z);
    set(root_yaw, keepYaw);

    set(r_leg_yaw,  0.0);
    set(r_hip_roll, 0.0);
    set(r_hip,     -0.04);
    set(r_knee,     0.04);
    set(r_ankle,    0.08);
    set(r_ankle_roll, 0.0);

    set(l_leg_yaw,  0.0);
    set(l_hip_roll, 0.0);
    set(l_hip,      0.04);
    set(l_knee,    -0.04);
    set(l_ankle,   -0.08);
    set(l_ankle_roll, 0.0);

    set(body_pitch, 0.05);
    set(body_yaw,   0.0);

    set(r_arm,  0.1);
    set(l_arm,  0.1);
    set(head_yaw,   0.0);
    set(head_pitch, 0.0);

    if(setActualQ){
        body->calcForwardKinematics();
    }
}

void RobotContext::lockRootOnFloor(bool correctActualQ,
                                   double keepX, double keepY, double keepYaw)
{
    if(!root_z) return;
    root_z->q_target() = STAND_ROOT_Z;

    if(correctActualQ){
        if(root_x) root_x->q_target() = keepX;
        if(root_y) root_y->q_target() = keepY;
        if(root_yaw) root_yaw->q_target() = keepYaw;
        const double err = STAND_ROOT_Z - root_z->q();
        if(std::abs(err) > 0.002){
            applyStandPose(true, keepX, keepY, keepYaw);
        }
    }
}

double RobotContext::clamp01(double x)
{
    if(x < 0.0) return 0.0;
    if(x > 1.0) return 1.0;
    return x;
}

double RobotContext::smoothstep(double x)
{
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
}

double RobotContext::blend(double from, double to, double s)
{
    return from + (to - from) * smoothstep(s);
}

double RobotContext::forwardRootY(double moveTime,
                                  double speed,
                                  double rootRampDur,
                                  double duration)
{
    if(moveTime <= 0.0){
        return 0.0;
    }
    const double tf = (moveTime >= duration) ? duration : moveTime;
    return speed * tf * smoothstep(tf / rootRampDur);
}
