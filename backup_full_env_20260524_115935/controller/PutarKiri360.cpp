#include "PutarKiri360.hpp"

void nodePutarKiri360(RobotContext& robot,
                      double t,
                      double duration,
                      double freezeX,
                      double freezeY,
                      double startYaw)
{
    double tt = t;
    if(tt < 0.0)      tt = 0.0;
    if(tt > duration) tt = duration;

    const double sTurn = RobotContext::smoothstep(tt / duration);
    const double rootYawCmd = startYaw + 2.0 * M_PI * sTurn;

    const double stepTime = 1.0;
    const double w = 2.0 * M_PI / stepTime;
    const double s = std::sin(w * tt);
    const double c = std::cos(w * tt);

    const double rightPhase = s;
    const double leftPhase  = s;

    const double rightLift = std::max(0.0, rightPhase);
    const double leftLift  = std::max(0.0, leftPhase);

    const double rightLiftPow = std::pow(rightLift, 0.50);
    const double leftLiftPow  = std::pow(leftLift, 0.50);

    const double hipAmp   = 0.080;
    const double kneeAmp  = 0.130;
    const double ankleAmp = 0.030;
    const double armAmp   = 0.20;

    robot.setRoot(freezeX, freezeY, 0.0, rootYawCmd);
    robot.keepYawNeutral();
    if(robot.root_yaw){
        robot.root_yaw->q_target() = rootYawCmd;
    }

    robot.setTarget(robot.r_hip,
                    robot.q0of(robot.r_hip) + hipAmp * rightPhase);
    robot.setTarget(robot.r_knee,
                    robot.q0of(robot.r_knee) + kneeAmp * rightLiftPow);
    robot.setTarget(robot.r_ankle,
                    robot.q0of(robot.r_ankle) - ankleAmp * rightPhase);

    robot.setTarget(robot.l_hip,
                    robot.q0of(robot.l_hip) + hipAmp * leftPhase);
    robot.setTarget(robot.l_knee,
                    robot.q0of(robot.l_knee) + kneeAmp * leftLiftPow);
    robot.setTarget(robot.l_ankle,
                    robot.q0of(robot.l_ankle) - ankleAmp * leftPhase);

    robot.setTarget(robot.l_arm,
                    robot.q0of(robot.l_arm) - armAmp * rightPhase);
    robot.setTarget(robot.r_arm,
                    robot.q0of(robot.r_arm) - armAmp * rightPhase);

    robot.setTarget(robot.body_pitch,
                    robot.q0of(robot.body_pitch) + 0.02 + 0.008 * c);
}
