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

    // Parameter utama lompat
    const double crouchDepth = 0.38;  // tekukan kaki lebih dalam  // jongkok lebih dalam untuk lompat tinggi  // semakin besar = jongkok lebih dalam
    const double jumpHeight  = 0.55;  // lompat tinggi  // SUPER HIGH JUMP ekstrem
    const double armAmp      = 1.05;  // ayunan tangan lebih besar  // ayunan tangan lebih kuat
    const double bodyAmp     = 0.32;  // tubuh lebih bungkuk saat lompat  // badan lebih condong saat take-off

    double zCmd = 0.0;
    double legFold = 0.0;
    double armSwing = 0.0;
    double bodyPitch = 0.0;

    if(tau < 0.25){
        // Fase 1: jongkok / ancang-ancang
        double a = tau / 0.25;

        legFold = crouchDepth * a;
        zCmd = -0.02 * a;
        armSwing = -armAmp * a;
        bodyPitch = bodyAmp * a;
    }
    else if(tau < 0.55){
        // Fase 2: dorong naik / take-off
        double a = (tau - 0.25) / 0.30;

        legFold = crouchDepth * (1.0 - a);
        zCmd = jumpHeight * std::sin(M_PI * a);
        armSwing = -armAmp + 2.0 * armAmp * a;
        bodyPitch = bodyAmp * (1.0 - a);
    }
    else if(tau < 0.80){
        // Fase 3: landing, lutut menekuk lagi
        double a = (tau - 0.55) / 0.25;

        legFold = crouchDepth * a;
        zCmd = 0.04 * std::sin(M_PI * (1.0 - a));
        armSwing = armAmp * (1.0 - a);
        bodyPitch = 0.04 * a;
    }
    else{
        // Fase 4: kembali stabil
        double a = (tau - 0.80) / 0.20;

        legFold = crouchDepth * (1.0 - a);
        zCmd = 0.0;
        armSwing = 0.0;
        bodyPitch = 0.04 * (1.0 - a);
    }

    // Root tetap di posisi X-Y, hanya Z naik-turun
    robot.setRoot(freezeX, freezeY, zCmd, 0.0);
    robot.keepYawNeutral();

    // Kedua kaki menekuk bersamaan
    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) - 0.75 * legFold);
    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) - 0.75 * legFold);

    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + 1.95 * legFold);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + 1.95 * legFold);

    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle) - 1.10 * legFold);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle) - 1.10 * legFold);

    // Tangan ikut ayun saat lompat
    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) + armSwing);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) + armSwing);

    // Badan condong sedikit saat ancang-ancang
    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + bodyPitch);

    return zCmd;
}
