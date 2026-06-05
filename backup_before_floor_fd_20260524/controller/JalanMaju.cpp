#include "JalanMaju.hpp"

double nodeJalanMaju(RobotContext& robot,
                     double t,
                     double duration,
                     double freezeX,
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

    // ─── amplitudes (walk forward = slightly bigger than in-place) ───
    const double hipAmp   = 0.40;
    const double kneeAmp  = 0.30;
    const double ankleAmp = 0.09;

    const double armAmp   = 0.42;
    const double bodyAmp  = 0.012;

    // Translate root forward.  Positive Y is forward in world frame
    // after the FORWARD_SIGN convention applied below.
    const double yCmd = rootYSign *
                        RobotContext::forwardRootY(t, speed, rootRampDur, duration);

    robot.setRoot(freezeX, yCmd, 0.0, 0.0);
    robot.keepYawNeutral();

    // ── RIGHT LEG ──
    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) + hipAmp * rightPhase);
    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + kneeAmp * rightLiftPow);
    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle)
                    - ankleAmp * rightPhase
                    - ankleAmp * 0.22 * rightLiftPow);

    // ── LEFT LEG ──
    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) + hipAmp * leftPhase);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + kneeAmp * leftLiftPow);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle)
                    - ankleAmp * leftPhase
                    - ankleAmp * 0.22 * leftLiftPow);

    // ── arms ──
    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) + armAmp * rightPhase);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) - armAmp * leftPhase);

    // ── body lean slightly into stride ──
    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + 0.03 + bodyAmp * c);

    return yCmd;
}
