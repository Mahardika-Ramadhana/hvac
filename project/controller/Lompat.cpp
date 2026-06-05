#include "Lompat.hpp"
#include <cmath>
#include <algorithm>

double nodeLompat(RobotContext& robot,
                  double t,
                  double duration,
                  double freezeX,
                  double freezeY,
                  double startYaw)
{
    double tau = t / duration;
    tau = std::max(0.0, std::min(1.0, tau));

    // zOffset relatif terhadap STAND_ROOT_Z (0.81 m), bukan tinggi absolut.
    const double crouchDepth = 0.22;
    const double jumpOffset  = 0.28;
    const double armAmp      = 0.75;
    const double bodyAmp     = 0.18;

    double zOffset = 0.0;
    double legFold = 0.0;
    double armSwing = 0.0;
    double bodyPitch = 0.0;

    if(tau < 0.22){
        // Fase 1: jongkok — pelvis sedikit turun, kaki menekuk
        const double a = tau / 0.22;
        legFold = crouchDepth * a;
        zOffset = -0.035 * a;
        armSwing = -armAmp * 0.35 * a;
        bodyPitch = bodyAmp * a;
    }
    else if(tau < 0.50){
        // Fase 2: take-off — pelvis naik di atas stand
        const double a = (tau - 0.22) / 0.28;
        legFold = crouchDepth * (1.0 - a);
        zOffset = jumpOffset * std::sin(M_PI * a);
        armSwing = -armAmp * 0.35 + armAmp * a;
        bodyPitch = bodyAmp * (1.0 - a);
    }
    else if(tau < 0.78){
        // Fase 3: landing — absorb impact, lutut menekuk
        const double a = (tau - 0.50) / 0.28;
        legFold = crouchDepth * 0.85 * a;
        zOffset = jumpOffset * 0.08 * (1.0 - a);
        armSwing = armAmp * 0.4 * (1.0 - a);
        bodyPitch = 0.06 * a;
    }
    else{
        // Fase 4: recovery ke stand penuh di floor
        const double a = (tau - 0.78) / 0.22;
        legFold = crouchDepth * 0.85 * (1.0 - a);
        zOffset = 0.0;
        armSwing = 0.0;
        bodyPitch = 0.06 * (1.0 - a);
    }

    robot.setRootWithZOffset(freezeX, freezeY, zOffset, startYaw);
    robot.keepYawNeutral();
    if(robot.root_yaw){ robot.root_yaw->q_target() = startYaw; }

    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) - 0.65 * legFold);
    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) - 0.65 * legFold);

    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + 1.55 * legFold);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + 1.55 * legFold);

    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle) - 0.85 * legFold);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle) - 0.85 * legFold);

    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) + armSwing);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) + armSwing);

    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + bodyPitch);

    return zOffset;
}
