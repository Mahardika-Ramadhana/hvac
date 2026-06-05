#include "JalanMundur.hpp"

double nodeJalanMundur(RobotContext& robot,
                       double t,
                       double duration,
                       double freezeX,
                       double yStart,
                       double rootYSign,
                       double speed,
                       double rootRampDur)
{
    const double stepTime = 1.20;
    const double w = 2.0 * M_PI / stepTime;

    const double s = std::sin(w * t);
    const double c = std::cos(w * t);

    const double rightPhase = s;
    const double leftPhase  = s;

    const double rightLift = std::max(0.0, rightPhase);
    const double leftLift  = std::max(0.0, leftPhase);

    const double liftGamma = 0.58;
    const double rightLiftPow = std::pow(rightLift, liftGamma);
    const double leftLiftPow  = std::pow(leftLift, liftGamma);

    const double hipAmp   = 0.40;
    const double kneeAmp  = 0.30;
    const double ankleAmp = 0.09;
    const double armAmp   = 0.40;
    const double bodyAmp  = 0.012;

    // Walk backward = subtract from yStart.
    const double yCmd = yStart -
                        rootYSign * RobotContext::forwardRootY(t, speed, rootRampDur, duration);

    robot.setRoot(freezeX, yCmd, 0.0, 0.0);
    robot.keepYawNeutral();

    // Hip phase reversed so legs visually step backward.
    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) - hipAmp * rightPhase);
    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + kneeAmp * rightLiftPow);
    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle)
                    + ankleAmp * rightPhase
                    - ankleAmp * 0.22 * rightLiftPow);

    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) - hipAmp * leftPhase);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + kneeAmp * leftLiftPow);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle)
                    + ankleAmp * leftPhase
                    - ankleAmp * 0.22 * leftLiftPow);

    // Arms also reversed for natural-looking back stride.
    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) - armAmp * rightPhase);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) + armAmp * leftPhase);

    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + 0.02 + bodyAmp * c);

    return yCmd;
}
