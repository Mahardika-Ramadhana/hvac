#include <cnoid/SimpleController>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>

using namespace cnoid;

class nyobarobotjalan : public SimpleController
{
    Body* body_ = nullptr;
    SimpleControllerIO* io_ = nullptr;

    Link* root_x = nullptr;
    Link* root_y = nullptr;
    Link* root_z = nullptr;
    Link* root_yaw = nullptr;

    Link* r_leg_yaw = nullptr;
    Link* l_leg_yaw = nullptr;
    Link* r_hip_roll = nullptr;
    Link* l_hip_roll = nullptr;
    Link* r_hip = nullptr;
    Link* l_hip = nullptr;
    Link* r_knee = nullptr;
    Link* l_knee = nullptr;
    Link* r_ankle = nullptr;
    Link* l_ankle = nullptr;
    Link* r_ankle_roll = nullptr;
    Link* l_ankle_roll = nullptr;

    Link* r_arm = nullptr;
    Link* l_arm = nullptr;

    Link* body_pitch = nullptr;
    Link* body_yaw = nullptr;

    Link* head_yaw = nullptr;
    Link* head_pitch = nullptr;

    double t = 0.0;
    double dt = 0.0;
    bool firstTick = true;

    double STAND_ROOT_Z = 0.81;

    double startY = 0.0;
    double startX = 0.0;
    double startYaw = 0.0;

    double freezeX = 0.0;
    double freezeY = 0.0;
    double freezeYaw = 0.0;
    double yAfterMaju = 0.0;
    double yAfterMundur = 0.0;

    std::vector<double> q0;

    // Timeline Configuration
    const bool USE_DIAM_AWAL       = true;
    const bool USE_JALAN_TEMPAT    = true;
    const bool USE_JALAN_MAJU      = true;
    const bool USE_JALAN_MUNDUR    = true;
    const bool USE_LOMPAT          = true;
    const bool USE_PUTAR_KIRI360   = true;
    const bool USE_DIAM_AKHIR      = true;

    const double T_DIAM_AWAL       = 2.0;
    const double T_JALAN_TEMPAT    = 3.0;
    const double T_JALAN_MAJU      = 15.0;
    const double T_JALAN_MUNDUR    = 5.0;
    const double T_LOMPAT          = 2.0;
    const double T_PUTAR_KIRI360   = 6.0;
    const double T_DIAM_AKHIR      = 4.0;

    const double floorMode         = true;
    const double rootYSign         = -1.0;
    const double walkSpeed         = 0.045;
    const double rootRampDur       = 1.5;

public:
    bool configure(SimpleControllerConfig* config) override
    {
        return true;
    }

    bool initialize(SimpleControllerIO* io) override
    {

        io_ = io;
        dt = io->timeStep();
        t = 0.0;
        firstTick = true;

        body_ = io->body();

        std::cout << "=== nyobarobotjalan (Clean Bipedal Walking Controller) ===" << std::endl;
        std::cout << "Joints: " << body_->numJoints() << std::endl;

        // Map joints by name
        auto getJoint = [&](const std::string& name) -> Link* {
            Link* j = body_->joint(name);
            if (!j) {
                if (name.rfind("_joint") == std::string::npos) {
                    j = body_->joint(name + "_joint");
                }
                if (!j && name.rfind("_link") == std::string::npos) {
                    j = body_->joint(name + "_link");
                }
            }
            return j;
        };

        root_x = getJoint("root_x");
        root_y = getJoint("root_y");
        root_z = getJoint("root_z");
        root_yaw = getJoint("root_yaw");

        r_leg_yaw = getJoint("right_knee_yaw_2");
        l_leg_yaw = getJoint("left_knee_yaw_1");
        r_hip_roll = getJoint("right_knee_roll_2");
        l_hip_roll = getJoint("left_knee_roll_1");

        r_hip = getJoint("right_knee_pitch_2");
        r_knee = getJoint("right_knee_pitch_4");
        r_ankle = getJoint("right_knee_pitch_6");

        l_hip = getJoint("left_knee_pitch_1");
        l_knee = getJoint("left_knee_pitch_3");
        l_ankle = getJoint("left_knee_pitch_5");

        r_ankle_roll = getJoint("right_knee_roll_6");
        l_ankle_roll = getJoint("left_knee_roll_5");

        r_arm = getJoint("right_arm_pitch_2");
        l_arm = getJoint("left_arm_pitch_1");

        body_pitch = getJoint("body_pitch");
        body_yaw = getJoint("body_yaw");

        head_yaw = getJoint("head_yaw");
        head_pitch = getJoint("head_pitch");

        // Set up q0 bias
        q0.assign(body_->numJoints(), 0.0);
        auto bias = [&](Link* j, double v){
            if(j && j->jointId() >= 0 && j->jointId() < (int)q0.size()) {
                q0[j->jointId()] += v;
            }
        };

        bias(r_knee,  0.10);
        bias(l_knee,  0.10);
        bias(r_hip,   0.05);
        bias(l_hip,   0.05);
        bias(r_ankle, 0.05);
        bias(l_ankle, 0.05);

        // Enable actuation in Position mode
        const int inputFlags = (int)Link::JointDisplacement | (int)Link::JointVelocity;
        for (int i = 0; i < body_->numJoints(); ++i) {
            Link* j = body_->joint(i);
            j->setActuationMode(Link::JointDisplacement);
            io_->enableInput(j, inputFlags);
            io_->enableOutput(j);
        }

        return true;
    }

    double q0of(Link* j) const
    {
        if(j && j->jointId() >= 0 && j->jointId() < (int)q0.size()){
            return q0[j->jointId()];
        }
        return 0.0;
    }

    void setTarget(Link* j, double q)
    {
        if(j){
            j->q_target() = q;
        }
    }

    void setRoot(double x, double y, double z, double yaw, bool lockHeight)
    {
        if(root_x)   root_x->q_target()   = x;
        if(root_y)   root_y->q_target()   = y;
        if(root_z && lockHeight){
            root_z->q_target() = STAND_ROOT_Z;
        }
        if(root_yaw) root_yaw->q_target() = yaw;
    }

    void setRootWithZOffset(double x, double y, double zOffset, double yaw)
    {
        if(root_x)   root_x->q_target()   = x;
        if(root_y)   root_y->q_target()   = y;
        if(root_z)   root_z->q_target()   = STAND_ROOT_Z + zOffset;
        if(root_yaw) root_yaw->q_target() = yaw;
    }

    void applyStandPose(bool setActualQ, double keepX, double keepY, double keepYaw)
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
            body_->calcForwardKinematics();
        }
    }

    void lockRootOnFloor(bool correctActualQ, double keepX, double keepY, double keepYaw)
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

    void resetAllTargets()
    {
        const int N = body_->numJoints();
        for(int i = 0; i < N; ++i){
            Link* j = body_->joint(i);
            j->q_target() = q0of(j);
        }
        if(body_yaw) body_yaw->q_target() = 0.0;
        if(body_pitch) body_pitch->q_target() = q0of(body_pitch);
    }

    static double clamp01(double x)
    {
        if(x < 0.0) return 0.0;
        if(x > 1.0) return 1.0;
        return x;
    }

    static double smoothstep(double x)
    {
        x = clamp01(x);
        return x * x * (3.0 - 2.0 * x);
    }

    static double forwardRootY(double moveTime, double speed, double rootRampDur, double duration)
    {
        const double tf = std::max(0.0, std::min(duration, moveTime));
        return speed * tf * smoothstep(tf / rootRampDur);
    }

    // Motion implementations
    void nodeJalanDitempat(double t_local, double fX, double fY, double sYaw)
    {
        const double stepTime = 1.15;
        const double w = 2.0 * M_PI / stepTime;

        const double s = std::sin(w * t_local);
        const double c = std::cos(w * t_local);

        const double rightPhase = s;
        const double leftPhase  = s;

        const double rightLift = std::max(0.0, rightPhase);
        const double leftLift  = std::max(0.0, leftPhase);

        const double liftGamma = 0.50;
        const double rightLiftPow = std::pow(rightLift, liftGamma);
        const double leftLiftPow  = std::pow(leftLift, liftGamma);

        const double hipAmp   = 0.40;
        const double kneeAmp  = 0.30;
        const double ankleAmp = 0.09;
        const double armAmp   = 0.80;
        const double bodyAmp  = 0.010;

        setRoot(fX, fY, 0.0, sYaw, true);
        if(root_yaw){ root_yaw->q_target() = sYaw; }

        setTarget(r_hip, q0of(r_hip) + hipAmp * rightPhase);
        setTarget(r_knee, q0of(r_knee) + kneeAmp * rightLiftPow);
        setTarget(r_ankle, q0of(r_ankle) - ankleAmp * rightPhase - 0.035 * rightLiftPow);

        setTarget(l_hip, q0of(l_hip) + hipAmp * leftPhase);
        setTarget(l_knee, q0of(l_knee) + kneeAmp * leftLiftPow);
        setTarget(l_ankle, q0of(l_ankle) - ankleAmp * leftPhase + 0.035 * leftLiftPow);

        setTarget(l_arm, q0of(l_arm) - armAmp * rightPhase);
        setTarget(r_arm, q0of(r_arm) - armAmp * leftPhase);

        setTarget(body_pitch, q0of(body_pitch) + 0.02 + bodyAmp * c);
    }

    double nodeJalanMaju(double t_local, double duration, double fX, double fY, double sYaw, double ySign, double speed, double rampDur)
    {
        const double stepTime = 1.20;
        const double w = 2.0 * M_PI / stepTime;

        const double s = std::sin(w * t_local);
        const double c = std::cos(w * t_local);

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
        const double armAmp   = 0.42;
        const double bodyAmp  = 0.012;

        const double yCmd = fY + ySign * forwardRootY(t_local, speed, rampDur, duration);

        setRoot(fX, yCmd, 0.0, sYaw, true);
        if(root_yaw){ root_yaw->q_target() = sYaw; }

        setTarget(r_hip, q0of(r_hip) + hipAmp * rightPhase);
        setTarget(r_knee, q0of(r_knee) + kneeAmp * rightLiftPow);
        setTarget(r_ankle, q0of(r_ankle) - ankleAmp * rightPhase - ankleAmp * 0.22 * rightLiftPow);

        setTarget(l_hip, q0of(l_hip) + hipAmp * leftPhase);
        setTarget(l_knee, q0of(l_knee) + kneeAmp * leftLiftPow);
        setTarget(l_ankle, q0of(l_ankle) - ankleAmp * leftPhase - ankleAmp * 0.22 * leftLiftPow);

        setTarget(l_arm, q0of(l_arm) - armAmp * rightPhase);
        setTarget(r_arm, q0of(r_arm) - armAmp * leftPhase);

        setTarget(body_pitch, q0of(body_pitch) + 0.03 + bodyAmp * c);

        return yCmd;
    }

    double nodeJalanMundur(double t_local, double duration, double fX, double yStart, double sYaw, double ySign, double speed, double rampDur)
    {
        const double stepTime = 1.20;
        const double w = 2.0 * M_PI / stepTime;

        const double s = std::sin(w * t_local);
        const double c = std::cos(w * t_local);

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

        const double yCmd = yStart - ySign * forwardRootY(t_local, speed, rampDur, duration);

        setRoot(fX, yCmd, 0.0, sYaw, true);
        if(root_yaw){ root_yaw->q_target() = sYaw; }

        setTarget(r_hip, q0of(r_hip) - hipAmp * rightPhase);
        setTarget(r_knee, q0of(r_knee) + kneeAmp * rightLiftPow);
        setTarget(r_ankle, q0of(r_ankle) + ankleAmp * rightPhase - ankleAmp * 0.22 * rightLiftPow);

        setTarget(l_hip, q0of(l_hip) - hipAmp * leftPhase);
        setTarget(l_knee, q0of(l_knee) + kneeAmp * leftLiftPow);
        setTarget(l_ankle, q0of(l_ankle) + ankleAmp * leftPhase - ankleAmp * 0.22 * leftLiftPow);

        setTarget(l_arm, q0of(l_arm) - armAmp * rightPhase);
        setTarget(r_arm, q0of(r_arm) - armAmp * leftPhase);

        setTarget(body_pitch, q0of(body_pitch) + 0.02 + bodyAmp * c);

        return yCmd;
    }

    double nodeLompat(double t_local, double duration, double fX, double fY, double sYaw)
    {
        double tau = t_local / duration;
        tau = std::max(0.0, std::min(1.0, tau));

        const double crouchDepth = 0.22;
        const double jumpOffset  = 0.28;
        const double armAmp      = 0.75;
        const double bodyAmp     = 0.18;

        double zOffset = 0.0;
        double legFold = 0.0;
        double armSwing = 0.0;
        double bodyPitch = 0.0;

        if(tau < 0.22){
            const double a = tau / 0.22;
            legFold = crouchDepth * a;
            zOffset = -0.035 * a;
            armSwing = -armAmp * 0.35 * a;
            bodyPitch = bodyAmp * a;
        }
        else if(tau < 0.50){
            const double a = (tau - 0.22) / 0.28;
            legFold = crouchDepth * (1.0 - a);
            zOffset = jumpOffset * std::sin(M_PI * a);
            armSwing = -armAmp * 0.35 + armAmp * a;
            bodyPitch = bodyAmp * (1.0 - a);
        }
        else if(tau < 0.78){
            const double a = (tau - 0.50) / 0.28;
            legFold = crouchDepth * 0.85 * a;
            zOffset = jumpOffset * 0.08 * (1.0 - a);
            armSwing = armAmp * 0.4 * (1.0 - a);
            bodyPitch = 0.06 * a;
        }
        else{
            const double a = (tau - 0.78) / 0.22;
            legFold = crouchDepth * 0.85 * (1.0 - a);
            zOffset = 0.0;
            armSwing = 0.0;
            bodyPitch = 0.06 * (1.0 - a);
        }

        setRootWithZOffset(fX, fY, zOffset, sYaw);
        if(root_yaw){ root_yaw->q_target() = sYaw; }

        setTarget(r_hip, q0of(r_hip) - 0.65 * legFold);
        setTarget(l_hip, q0of(l_hip) - 0.65 * legFold);

        setTarget(r_knee, q0of(r_knee) + 1.55 * legFold);
        setTarget(l_knee, q0of(l_knee) + 1.55 * legFold);

        setTarget(r_ankle, q0of(r_ankle) - 0.85 * legFold);
        setTarget(l_ankle, q0of(l_ankle) - 0.85 * legFold);

        setTarget(l_arm, q0of(l_arm) + armSwing);
        setTarget(r_arm, q0of(r_arm) + armSwing);

        setTarget(body_pitch, q0of(body_pitch) + bodyPitch);

        return zOffset;
    }

    void nodePutarKiri360(double t_local, double duration, double fX, double fY, double sYaw)
    {
        double tt = t_local;
        if(tt < 0.0)      tt = 0.0;
        if(tt > duration) tt = duration;

        const double sTurn = smoothstep(tt / duration);
        const double rootYawCmd = sYaw + 2.0 * M_PI * sTurn;

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

        setRoot(fX, fY, 0.0, rootYawCmd, true);
        if(root_yaw){
            root_yaw->q_target() = rootYawCmd;
        }

        setTarget(r_hip, q0of(r_hip) + hipAmp * rightPhase);
        setTarget(r_knee, q0of(r_knee) + kneeAmp * rightLiftPow);
        setTarget(r_ankle, q0of(r_ankle) - ankleAmp * rightPhase);

        setTarget(l_hip, q0of(l_hip) + hipAmp * leftPhase);
        setTarget(l_knee, q0of(l_knee) + kneeAmp * leftLiftPow);
        setTarget(l_ankle, q0of(l_ankle) - ankleAmp * leftPhase);

        setTarget(l_arm, q0of(l_arm) - armAmp * rightPhase);
        setTarget(r_arm, q0of(r_arm) - armAmp * rightPhase);

        setTarget(body_pitch, q0of(body_pitch) + 0.02 + 0.008 * c);
    }

    bool isActiveWindow(bool enabled, double& cursor, double duration, double& localT)
    {
        const double start = cursor;
        const double end   = cursor + duration;
        cursor = end;
        if(!enabled) return false;
        if(t < start || t >= end) return false;
        localT = t - start;
        return true;
    }

    bool control() override
    {
        if (firstTick) {
            startX = root_x ? root_x->q() : 0.0;
            startY = root_y ? root_y->q() : 0.0;
            startYaw = root_yaw ? root_yaw->q() : 0.0;

            // Re-apply stand on first sim tick to lock pose
            applyStandPose(true, startX, startY, startYaw);

            freezeX = startX;
            freezeY = startY;
            freezeYaw = startYaw;

            firstTick = false;
            return true;
        }

        t += dt;
        resetAllTargets();

        double cursor = 0.0;
        double localT = 0.0;

        bool dispatched = false;
        bool isJumping = false;
        bool isHolding = false;
        double holdX = freezeX;
        double holdY = freezeY;
        double holdYaw = freezeYaw;

        const double yawAfterLompat = freezeYaw;
        const double yawAfterPutarKiri360 = yawAfterLompat + 2.0 * M_PI;
        const double yawFinal = yawAfterPutarKiri360;

        if (isActiveWindow(USE_DIAM_AWAL, cursor, T_DIAM_AWAL, localT)) {
            holdX = freezeX; holdY = freezeY; holdYaw = freezeYaw;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }
        if (!dispatched &&
            isActiveWindow(USE_JALAN_TEMPAT, cursor, T_JALAN_TEMPAT, localT)) {
            nodeJalanDitempat(localT, freezeX, freezeY, freezeYaw);
            dispatched = true;
        }
        if (!dispatched &&
            isActiveWindow(USE_JALAN_MAJU, cursor, T_JALAN_MAJU, localT)) {
            yAfterMaju = nodeJalanMaju(localT, T_JALAN_MAJU,
                                       freezeX, freezeY, freezeYaw, rootYSign, walkSpeed, rootRampDur);
            dispatched = true;
        } else if (USE_JALAN_MAJU) {
            yAfterMaju = freezeY + rootYSign * forwardRootY(T_JALAN_MAJU, walkSpeed, rootRampDur, T_JALAN_MAJU);
        } else {
            yAfterMaju = freezeY;
        }
        if (!dispatched &&
            isActiveWindow(USE_JALAN_MUNDUR, cursor, T_JALAN_MUNDUR, localT)) {
            yAfterMundur = nodeJalanMundur(localT, T_JALAN_MUNDUR,
                                           freezeX, yAfterMaju, freezeYaw, rootYSign, walkSpeed, rootRampDur);
            dispatched = true;
        } else {
            yAfterMundur = yAfterMaju;
        }
        if (!dispatched &&
            isActiveWindow(USE_LOMPAT, cursor, T_LOMPAT, localT)) {
            nodeLompat(localT, T_LOMPAT, freezeX, yAfterMundur, yawAfterLompat);
            isJumping = true;
            dispatched = true;
        }
        if (!dispatched &&
            isActiveWindow(USE_PUTAR_KIRI360, cursor, T_PUTAR_KIRI360, localT)) {
            nodePutarKiri360(localT, T_PUTAR_KIRI360, freezeX, yAfterMundur, yawAfterLompat);
            dispatched = true;
        }
        if (!dispatched &&
            isActiveWindow(USE_DIAM_AKHIR, cursor, T_DIAM_AKHIR, localT)) {
            holdX = freezeX; holdY = yAfterMundur; holdYaw = yawFinal;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }
        if (!dispatched) {
            holdX = freezeX; holdY = yAfterMundur; holdYaw = yawFinal;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
        }

        // Apply stand locks during hold periods
        if (isHolding) {
            applyStandPose(true, holdX, holdY, holdYaw);
            lockRootOnFloor(true, holdX, holdY, holdYaw);
        } else if (floorMode && !isJumping && root_z) {
            root_z->q_target() = STAND_ROOT_Z;
        }

        body_->calcForwardKinematics();

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(nyobarobotjalan)
