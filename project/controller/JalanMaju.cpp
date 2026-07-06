#include "JalanMaju.hpp"
#include "RobotContext.hpp"
#include <iostream>
#include <cmath>
#include <cnoid/JointPath>

using namespace cnoid;

// Stand pose angles matching .cnoid jointDisplacements (designed for floor contact at Z=0.38)
static void applyDirectStandAngles(RobotContext& robot)
{
    auto tgt = [](Link* j, double v){ if(j) j->q_target() = v; };
    tgt(robot.r_leg_yaw,    0.0);
    tgt(robot.r_hip_roll,   0.0);
    tgt(robot.r_hip,       -0.04);
    tgt(robot.r_knee,       0.04);
    tgt(robot.r_ankle,      0.08);
    tgt(robot.r_ankle_roll, 0.0);

    tgt(robot.l_leg_yaw,    0.0);
    tgt(robot.l_hip_roll,   0.0);
    tgt(robot.l_hip,        0.04);
    tgt(robot.l_knee,      -0.04);
    tgt(robot.l_ankle,     -0.08);
    tgt(robot.l_ankle_roll, 0.0);
}

double nodeJalanMaju(RobotContext& robot, double localT, double duration,
                     double freezeX, double startY, double startYaw, double stepY, double cycleTime)
{
    double phase = fmod(localT, cycleTime) / cycleTime; // 0 to 1

    double stepLength = stepY;
    double stepHeight = 0.04; // Slightly reduced for stability
    double swayAmp    = 0.09; // 9cm perfectly shifts CoM over the support foot (0.03 +- 0.09 = 0.12 and -0.06)

    // Actual foot positions in base frame (from IK INIT debug):
    // Right: (-0.0627, -0.0341, -0.3311)  Left: (0.1233, -0.0118, -0.3489)
    // Use Z slightly above these to keep knee bent during walk
    // Use Z slightly above these to keep knee bent during walk
    const double FOOT_Z       = -0.3489;
    Vector3 rFootNeutral(-0.0626, -0.034, FOOT_Z);
    Vector3 lFootNeutral( 0.1233, -0.012, FOOT_Z);

    Vector3 rTarget = rFootNeutral;
    Vector3 lTarget = lFootNeutral;


    // Lateral sway — shifts targets in X so CoM stays over support foot
    double sway = -sin(phase * 2.0 * M_PI) * swayAmp;
    rTarget.x() += sway;
    lTarget.x() += sway;

    // Step motion
    double rStep = 0, lStep = 0, rLift = 0, lLift = 0;
    if (phase < 0.5) {
        double sub = phase * 2.0;
        rStep = -stepLength/2.0 + stepLength * sub;
        lStep =  stepLength/2.0 - stepLength * sub;
        rLift = sin(sub * M_PI) * stepHeight;
    } else {
        double sub = (phase - 0.5) * 2.0;
        lStep = -stepLength/2.0 + stepLength * sub;
        rStep =  stepLength/2.0 - stepLength * sub;
        lLift = sin(sub * M_PI) * stepHeight;
    }

    rTarget.y() -= rStep;
    rTarget.z() += rLift;
    lTarget.y() -= lStep;
    lTarget.z() += lLift;

    // Update forward kinematics so IK Jacobian is based on current state
    robot.body->calcForwardKinematics();

    bool ikOk = false;
    if (robot.ikRightLeg && robot.ikLeftLeg) {
        cnoid::Isometry3 T_r;
        T_r.linear() = robot.rFootRot0;
        T_r.translation() = rTarget;
        bool rOk = robot.ikRightLeg->calcInverseKinematics(T_r);

        cnoid::Isometry3 T_l;
        T_l.linear() = robot.lFootRot0;
        T_l.translation() = lTarget;
        bool lOk = robot.ikLeftLeg->calcInverseKinematics(T_l);
        ikOk = rOk && lOk;

        // Print IK result every 1s (phase near 0)
        static double lastPrint = -1.0;
        if (localT - lastPrint > 1.0) {
            lastPrint = localT;
            std::cout << "[IK] t=" << localT
                      << " rOk=" << rOk << " lOk=" << lOk
                      << " rTarget=(" << rTarget.transpose() << ")"
                      << " lTarget=(" << lTarget.transpose() << ")" << std::endl;
        }

        if (rOk) {
            for (int i = 0; i < robot.ikRightLeg->numJoints(); ++i) {
                Link* j = robot.ikRightLeg->joint(i);
                robot.setTarget(j, j->q());
            }
        }
        if (lOk) {
            for (int i = 0; i < robot.ikLeftLeg->numJoints(); ++i) {
                Link* j = robot.ikLeftLeg->joint(i);
                robot.setTarget(j, j->q());
            }
        }
    }

    // Fallback: if IK failed, use direct stand angles
    if (!ikOk) {
        applyDirectStandAngles(robot);
    }

    // Arm swing (anti-phase with legs)
    if (robot.l_arm) robot.l_arm->q_target() = 0.25 + lStep * 4.0;
    if (robot.r_arm) robot.r_arm->q_target() = 0.25 + rStep * 4.0;

    // Slight forward body lean during walk
    if (robot.body_pitch) robot.body_pitch->q_target() = 0.05;

    return startY + (duration / cycleTime) * stepLength;
}
