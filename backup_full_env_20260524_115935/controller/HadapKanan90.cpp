#include "HadapKanan90.hpp"

void nodeHadapKanan90(RobotContext& robot,
                      double t,
                      double duration,
                      double freezeX,
                      double freezeY,
                      double startYaw)
{
    double tt = t;
    if(tt < 0.0)        tt = 0.0;
    if(tt > duration)   tt = duration;

    const double sTurn = RobotContext::smoothstep(tt / duration);

    // Use the virtual root_yaw_joint to rotate the whole base.
    // Hadap kanan = yaw negative (right-hand turn around +Z).
    const double targetYaw = -M_PI_2;          // -90°
    const double rootYawCmd = startYaw + targetYaw * sTurn;

    // Helper marching gait (small)
    const double stepTime = 1.15;
    const double w = 2.0 * M_PI / stepTime;
    const double s = std::sin(w * tt);
    const double c = std::cos(w * tt);

    const double rightPhase = s;
    const double leftPhase  = s;

    const double rightLift = std::max(0.0, rightPhase);
    const double leftLift  = std::max(0.0, leftPhase);

    const double rightLiftPow = std::pow(rightLift, 0.50);
    const double leftLiftPow  = std::pow(leftLift, 0.50);

    const double hipAmp   = 0.060;
    const double kneeAmp  = 0.110;
    const double ankleAmp = 0.025;
    const double armAmp   = 0.18;
    const double bodyAmp  = 0.004;

    robot.setRoot(freezeX, freezeY, 0.0, rootYawCmd);
    robot.keepYawNeutral();           // re-overwrite root_yaw below

    if(robot.root_yaw){
        robot.root_yaw->q_target() = rootYawCmd;
    }

    // RIGHT leg march
    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) + hipAmp * rightPhase);
    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + kneeAmp * rightLiftPow);
    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle)
                    - ankleAmp * rightPhase
                    + 0.012 * rightLiftPow);

    // LEFT leg march
    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) + hipAmp * leftPhase);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + kneeAmp * leftLiftPow);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle)
                    - ankleAmp * leftPhase
                    + 0.012 * leftLiftPow);

    // Arm sway (both shoulders same direction biases toward the turn)
    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) - armAmp * rightPhase);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) - armAmp * rightPhase);

    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + 0.02 + bodyAmp * c);
}
