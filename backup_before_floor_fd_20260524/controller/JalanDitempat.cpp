#include "JalanDitempat.hpp"

void nodeJalanDitempat(RobotContext& robot,
                       double t,
                       double freezeX,
                       double freezeY)
{
    const double stepTime = 1.15;
    const double w = 2.0 * M_PI / stepTime;

    const double s = std::sin(w * t);
    const double c = std::cos(w * t);

    const double rightPhase = s;
    const double leftPhase  = s;

    const double rightLift = std::max(0.0, rightPhase);
    const double leftLift  = std::max(0.0, leftPhase);

    const double liftGamma = 0.50;
    const double rightLiftPow = std::pow(rightLift, liftGamma);
    const double leftLiftPow  = std::pow(leftLift, liftGamma);

    // ───── motion amplitudes ─────
    const double hipAmp   = 0.40;
    const double kneeAmp  = 0.30;
    const double ankleAmp = 0.09;

    const double armAmp   = 0.30;
    const double bodyAmp  = 0.010;

    // Root locked – walking in place
    robot.setRoot(freezeX, freezeY, 0.0, 0.0);
    robot.keepYawNeutral();

    // ───── RIGHT LEG ─────
    // Knee bends POSITIVELY (q0 + kneeAmp * lift) so the lower leg
    // folds under the body the way a human knee does.
    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) + hipAmp * rightPhase);
    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + kneeAmp * rightLiftPow);
    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle)
                    - ankleAmp * rightPhase
                    - 0.035 * rightLiftPow);

    // ───── LEFT LEG ─────
    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) + hipAmp * leftPhase);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + kneeAmp * leftLiftPow);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle)
                    - ankleAmp * leftPhase
                    + 0.035 * leftLiftPow);

    // ───── arms swing opposite to legs ─────
    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) + armAmp * rightPhase);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) + armAmp * leftPhase);

    // ───── body bob ─────
    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + 0.02 + bodyAmp * c);
}
