#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <memory>
#include <cmath>
#include <algorithm>
#include <fstream>

using namespace cnoid;

class jalan_lari : public SimpleController
{
    Body* body_ = nullptr;
    SimpleControllerIO* io_ = nullptr;
    
    std::ofstream debugLog;

    Link* root_x = nullptr;
    Link* root_y = nullptr;
    Link* root_z = nullptr;
    Link* root_yaw = nullptr;

    std::shared_ptr<cnoid::JointPath> ikRightLeg;
    std::shared_ptr<cnoid::JointPath> ikLeftLeg;
    cnoid::Matrix3 rFootRot0;
    cnoid::Matrix3 lFootRot0;

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
    Link* r_elbow = nullptr;
    Link* l_elbow = nullptr;

    Link* body_pitch = nullptr;
    Link* body_yaw = nullptr;

    Link* head_yaw = nullptr;
    Link* head_pitch = nullptr;

    double t = 0.0;
    double dt = 0.0;
    bool firstTick = true;

    double STAND_ROOT_Z = 0.365;

    double startY = 0.0;
    double startX = 0.0;
    double startYaw = 0.0;

    double freezeX = 0.0;
    double freezeY = 0.0;
    double freezeYaw = 0.0;
    double yAfterMaju = 0.0;
    double yAfterLari = 0.0;

    std::vector<double> q0;

    // Timeline Configuration
    const bool USE_DIAM_AWAL       = true;
    const bool USE_JALAN_MAJU      = false;
    const bool USE_DIAM_TRANSISI   = false;
    const bool USE_LARI_MAJU       = true;
    const bool USE_DIAM_AKHIR      = true;

    const double T_DIAM_AWAL       = 0.2;
    const double T_JALAN_MAJU      = 0.0;
    const double T_DIAM_TRANSISI   = 0.0;
    const double T_LARI_MAJU       = 50.0;
    const double T_DIAM_AKHIR      = 1.0;

    const double floorMode         = true;
    const double rootYSign         = -1.0;
    const double walkSpeed         = 0.045;
    const double runSpeed          = 0.85;
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

        std::cout << "=== jalan_lari (Bipedal Walk & Run Controller) ===" << std::endl;
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

        root_x = getJoint("root_x_joint");
        root_y = getJoint("root_y_joint");
        root_z = getJoint("root_z_joint");
        root_yaw = getJoint("root_yaw_joint");

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
        r_elbow = getJoint("right_arm_roll_4");
        l_elbow = getJoint("left_arm_roll_3");

        body_pitch = getJoint("body_pitch");
        body_yaw = getJoint("body_yaw");

        head_yaw = getJoint("head_yaw");
        head_pitch = getJoint("head_pitch");

        // Set up q0 to exactly match applyStandPose (FLAT FEET)
        q0.assign(body_->numJoints(), 0.0);
        auto set_q0 = [&](Link* j, double v){
            if(j && j->jointId() >= 0 && j->jointId() < (int)q0.size()) {
                q0[j->jointId()] = v;
            }
        };

        set_q0(r_hip, -0.04);
        set_q0(r_knee, 0.04);
        set_q0(r_ankle, 0.0);

        set_q0(l_hip, 0.04);
        set_q0(l_knee, -0.04);
        set_q0(l_ankle, 0.0);

        set_q0(body_pitch, 0.05);
        set_q0(r_arm, 0.1);
        set_q0(l_arm, 0.1);

        // Enable actuation in Position mode
        const int inputFlags = (int)Link::JointDisplacement | (int)Link::JointVelocity;
        for (int i = 0; i < body_->numJoints(); ++i) {
            Link* j = body_->joint(i);
            j->setActuationMode(Link::JointDisplacement);
            io_->enableInput(j, inputFlags);
            io_->enableOutput(j);
        }

        Link* baseLink = body_->link("base_link");
        if(baseLink){
            io_->enableInput(baseLink, Link::LinkPosition);
            body_->calcForwardKinematics();
            ikRightLeg = cnoid::JointPath::getCustomPath(baseLink, r_ankle_roll ? r_ankle_roll : r_ankle);
            ikLeftLeg = cnoid::JointPath::getCustomPath(baseLink, l_ankle_roll ? l_ankle_roll : l_ankle);
            if(ikRightLeg) rFootRot0 = baseLink->R().transpose() * ikRightLeg->endLink()->R();
            if(ikLeftLeg) lFootRot0 = baseLink->R().transpose() * ikLeftLeg->endLink()->R();
        }
        
        debugLog.open("/home/dika/projects/hvac/fall_debug.csv", std::ios::out);
        if(debugLog.is_open()){
            debugLog << "time,isFallen,recoveryState,recoveryTimer,up_z,isProne,r_hip_tgt,r_knee_tgt,r_ankle_tgt,r_arm_tgt\n";
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

    void setRoot(double x, double y, double z, double yaw, bool lockHeight = true)
    {
        // BENAR-BENAR JANGAN DISENTUH (SIM-TO-REAL FREE FALL)
    }

    void applyStandPose(bool setActualQ, double keepX, double keepY, double keepZ, double keepYaw)
    {
        auto set = [&](Link* j, double v){
            if(!j) return;
            if(setActualQ) j->q() = v;
            j->q_target() = v;
        };

        // Biarkan joint virtual tetap pada posisi awalnya (rigid dengan root_x_link)
        // Dilarang memaksa target menjadi 0 karena akan merusak posisi pinggul!

        set(r_leg_yaw,  0.0);
        set(r_hip_roll, 0.0);
        set(r_hip,     -0.04);
        set(r_knee,     0.04);
        set(r_ankle,    0.0);
        set(r_ankle_roll, 0.0);

        set(l_leg_yaw,  0.0);
        set(l_hip_roll, 0.0);
        set(l_hip,      0.04);
        set(l_knee,    -0.04);
        set(l_ankle,    0.0);
        set(l_ankle_roll, 0.0);

        set(body_pitch, 0.05); // Match exactly with .cnoid to prevent physics explosion
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
        // NO MAGIC ANCHOR
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

    // Walking gait generator
    double nodeJalanMaju(double t_local, double duration, double fX, double fY, double sYaw, double ySign, double speed, double rampDur)
    {
        const double stepTime = 1.20;
        const double w = 2.0 * M_PI / stepTime;

        const double s = std::sin(w * t_local);
        const double c = std::cos(w * t_local);

        const double rightPhase = s;
        const double leftPhase  = -s;

        const double rightLift = std::max(0.0, rightPhase);
        const double leftLift  = std::max(0.0, leftPhase);

        const double liftGamma = 0.58;
        const double rightLiftPow = std::pow(rightLift, liftGamma);
        const double leftLiftPow  = std::pow(leftLift, liftGamma);

        const double hipAmp   = 0.40;
        const double kneeAmp  = 0.70; // Increase from 0.30
        const double ankleAmp = 0.09;
        const double armAmp   = 0.60; // Increase from 0.42
        const double elbowAmp = 0.50; // New elbow swing
        const double bodyAmp  = 0.012;

        const double yCmd = fY + ySign * forwardRootY(t_local, speed, rampDur, duration);

        setRoot(fX, yCmd, 0.0, sYaw, true);
        if(root_yaw){ root_yaw->q_target() = sYaw; }

        setTarget(r_hip, q0of(r_hip) + hipAmp * rightPhase);
        setTarget(r_knee, q0of(r_knee) - kneeAmp * rightLiftPow);
        setTarget(r_ankle, q0of(r_ankle) - ankleAmp * rightPhase + ankleAmp * 0.22 * rightLiftPow);

        setTarget(l_hip, q0of(l_hip) - hipAmp * leftPhase);
        setTarget(l_knee, q0of(l_knee) - kneeAmp * leftLiftPow);
        setTarget(l_ankle, q0of(l_ankle) + ankleAmp * leftPhase + ankleAmp * 0.22 * leftLiftPow);

        setTarget(l_arm, q0of(l_arm) + armAmp * rightPhase);
        setTarget(r_arm, q0of(r_arm) + armAmp * leftPhase);
        if(l_elbow) setTarget(l_elbow, q0of(l_elbow) + elbowAmp * std::max(0.0, rightPhase));
        if(r_elbow) setTarget(r_elbow, q0of(r_elbow) + elbowAmp * std::max(0.0, leftPhase));

        setTarget(body_pitch, q0of(body_pitch) + 0.03 + bodyAmp * c);

        return yCmd;
    }

    // Real ZMP (LIPM) Based Walking Gait Generator
    double nodeLariMaju(double t_local, double duration, double fX, double fY, double sYaw, double ySign, double speed, double rampDur)
    {
        const double stepTime = 0.35;
        const double Tc = sqrt(0.35 / 9.81); // LIPM Time constant (Zc = 35cm)
        
        double phase = fmod(t_local, stepTime * 2.0) / (stepTime * 2.0); // 0 to 1 for full cycle (2 steps)
        
        double stepLength = 0.08 * std::min(1.0, t_local / 2.0); // Start slow
        double stepHeight = 0.04;
        
        // Target ZMP placement (stance foot)
        double zmpAmp = 0.06; // 6cm sway left/right
        
        // Calculate Analytical LIPM CoM Sway (approximate symmetric ZMP tracking)
        // This ensures the Zero Moment Point stays within the support polygon
        double sway = -sin(phase * 2.0 * M_PI) * zmpAmp * 0.8; // CoM swaying
        
        const double FOOT_Z = -0.34;
        cnoid::Vector3 rTarget(-0.062, -0.034, FOOT_Z);
        cnoid::Vector3 lTarget( 0.123, -0.012, FOOT_Z);
        
        // CoM Shift relative to feet (or feet relative to CoM)
        rTarget.x() += sway;
        lTarget.x() += sway;
        
        double rStep = 0, lStep = 0, rLift = 0, lLift = 0;
        if (phase < 0.5) { // Right foot swinging
            double sub = phase * 2.0;
            rStep = -stepLength/2.0 + stepLength * sub;
            lStep =  stepLength/2.0 - stepLength * sub;
            rLift = sin(sub * M_PI) * stepHeight;
        } else { // Left foot swinging
            double sub = (phase - 0.5) * 2.0;
            lStep = -stepLength/2.0 + stepLength * sub;
            rStep =  stepLength/2.0 - stepLength * sub;
            lLift = sin(sub * M_PI) * stepHeight;
        }
        
        rTarget.y() -= rStep;
        rTarget.z() += rLift;
        lTarget.y() -= lStep;
        lTarget.z() += lLift;
        
        // Execute Inverse Kinematics for ZMP-stable pose
        if(ikRightLeg && ikLeftLeg) {
            cnoid::Isometry3 T_r;
            T_r.linear() = rFootRot0;
            T_r.translation() = rTarget;
            bool rOk = ikRightLeg->calcInverseKinematics(T_r);
            
            cnoid::Isometry3 T_l;
            T_l.linear() = lFootRot0;
            T_l.translation() = lTarget;
            bool lOk = ikLeftLeg->calcInverseKinematics(T_l);
            
            if (rOk) {
                for (int i = 0; i < ikRightLeg->numJoints(); ++i) {
                    Link* j = ikRightLeg->joint(i);
                    setTarget(j, j->q());
                }
            }
            if (lOk) {
                for (int i = 0; i < ikLeftLeg->numJoints(); ++i) {
                    Link* j = ikLeftLeg->joint(i);
                    setTarget(j, j->q());
                }
            }
        }
        
        // Arm Swing (Counter-balance)
        double armPhase = sin(phase * 2.0 * M_PI);
        if(r_arm) setTarget(r_arm, q0of(r_arm) - armPhase * 0.2);
        if(l_arm) setTarget(l_arm, q0of(l_arm) + armPhase * 0.2);
        
        // Body Pitch for forward momentum
        setTarget(body_pitch, q0of(body_pitch) + 0.05);
        
        return fY; // Not actively setting root_y via kinematics
    }

    bool isFallen = false;
    int recoveryState = 0;
    double recoveryTimer = 0.0;
    bool isProne = false;
    std::vector<double> q_fall;

    bool control() override
    {
        if(isFallen){
            recoveryTimer += dt;
            
            if (recoveryState == 0) {
                // Wait for 2 seconds to settle
                if (recoveryTimer > 2.0) {
                    recoveryState = 1;
                    recoveryTimer = 0.0;
                    Link* baseLink = body_->link("base_link");
                    if(baseLink){
                        isProne = (baseLink->R()(2,0) < 0.0); // Z-component of Local X < 0 means face down
                    }
                    std::cout << "[DEBUG-BANGKIT] Memulai Tahap 1. Tipe Jatuh: " << (isProne ? "Tengkurap" : "Telentang") << std::endl;
                }
            }
            else if (recoveryState == 1) {
                // Stage 1: Retract arms and legs (Kneeling)
                double duration = 4.0; // Diperlambat dari 2.0 ke 4.0 detik
                double progress = std::min(1.0, recoveryTimer / duration);
                double smooth = progress * progress * (3.0 - 2.0 * progress);
                
                if (isProne) {
                    setTarget(r_hip, q_fall[r_hip->jointId()] * (1-smooth) + (-1.5) * smooth);
                    setTarget(l_hip, q_fall[l_hip->jointId()] * (1-smooth) + (1.5) * smooth);
                    setTarget(r_knee, q_fall[r_knee->jointId()] * (1-smooth) + (2.0) * smooth);
                    setTarget(l_knee, q_fall[l_knee->jointId()] * (1-smooth) + (-2.0) * smooth);
                    setTarget(r_arm, q_fall[r_arm->jointId()] * (1-smooth) + (1.5) * smooth);
                    setTarget(l_arm, q_fall[l_arm->jointId()] * (1-smooth) + (-1.5) * smooth);
                    if (r_elbow) setTarget(r_elbow, q_fall[r_elbow->jointId()] * (1-smooth) + (-1.0) * smooth);
                    if (l_elbow) setTarget(l_elbow, q_fall[l_elbow->jointId()] * (1-smooth) + (1.0) * smooth);
                } else {
                    // Supine (face up) get up sequence (roll over first)
                    setTarget(r_hip, q_fall[r_hip->jointId()] * (1-smooth) + (0.0) * smooth);
                    setTarget(l_hip, q_fall[l_hip->jointId()] * (1-smooth) + (0.0) * smooth);
                    setTarget(r_arm, q_fall[r_arm->jointId()] * (1-smooth) + (-1.5) * smooth);
                    setTarget(l_arm, q_fall[l_arm->jointId()] * (1-smooth) + (1.5) * smooth);
                }
                
                if (recoveryTimer > 4.5) { // Jeda ekstra 0.5 detik
                    std::cout << "[DEBUG-BANGKIT] Masuk Tahap 2 (Jongkok / Straighten Torso)" << std::endl;
                    recoveryState = 2;
                    recoveryTimer = 0.0;
                }
            }
            else if (recoveryState == 2) {
                // Stage 2: Straighten torso, plant feet
                double duration = 4.0;
                double progress = std::min(1.0, recoveryTimer / duration);
                double smooth = progress * progress * (3.0 - 2.0 * progress);
                
                if (isProne) {
                    setTarget(r_hip, (-1.5) * (1-smooth) + (-0.5) * smooth);
                    setTarget(l_hip, (1.5) * (1-smooth) + (0.5) * smooth);
                    setTarget(r_knee, 2.0);
                    setTarget(l_knee, -2.0);
                    setTarget(r_ankle, -1.0 * smooth);
                    setTarget(l_ankle, 1.0 * smooth);
                    setTarget(r_arm, (1.5) * (1-smooth) + (-0.5) * smooth);
                    setTarget(l_arm, (-1.5) * (1-smooth) + (0.5) * smooth);
                } else {
                    // Supine: Try to curl and roll over
                    setTarget(r_hip, -1.5 * smooth);
                    setTarget(l_hip, 1.5 * smooth);
                    setTarget(r_knee, 2.0 * smooth);
                    setTarget(l_knee, -2.0 * smooth);
                }
                
                if (recoveryTimer > 4.5) {
                    std::cout << "[DEBUG-BANGKIT] Masuk Tahap 3 (Berdiri / Stand Up)" << std::endl;
                    recoveryState = 3;
                    recoveryTimer = 0.0;
                }
            }
            else if (recoveryState == 3) {
                // Stage 3: Stand up (interpolate to q0)
                double duration = 4.0;
                double progress = std::min(1.0, recoveryTimer / duration);
                double smooth = progress * progress * (3.0 - 2.0 * progress);
                
                if (isProne) {
                    setTarget(r_hip, (-0.5) * (1-smooth) + q0of(r_hip) * smooth);
                    setTarget(l_hip, (0.5) * (1-smooth) + q0of(l_hip) * smooth);
                    setTarget(r_knee, 2.0 * (1-smooth) + q0of(r_knee) * smooth);
                    setTarget(l_knee, -2.0 * (1-smooth) + q0of(l_knee) * smooth);
                    setTarget(r_ankle, -1.0 * (1-smooth) + q0of(r_ankle) * smooth);
                    setTarget(l_ankle, 1.0 * (1-smooth) + q0of(l_ankle) * smooth);
                    setTarget(r_arm, (-0.5) * (1-smooth) + q0of(r_arm) * smooth);
                    setTarget(l_arm, (0.5) * (1-smooth) + q0of(l_arm) * smooth);
                    if (r_elbow) setTarget(r_elbow, -1.0 * (1-smooth) + 0.0 * smooth);
                    if (l_elbow) setTarget(l_elbow, 1.0 * (1-smooth) + 0.0 * smooth);
                } else {
                    // Supine: finish roll over to prone, let it detect fall again
                    if (progress == 1.0) {
                        isFallen = false; // Reset to let it detect prone next tick
                    }
                }
                
                if (recoveryTimer > 5.0) {
                    std::cout << "[DEBUG-BANGKIT] Selesai! Kembali ke mode jalan/lari." << std::endl;
                    isFallen = false;
                    recoveryState = 0;
                    recoveryTimer = 0.0;
                    firstTick = true; // Re-initialize walking params
                }
            }
            
            if(debugLog.is_open()){
                Link* baseLink = body_->link("base_link");
                double up_z = baseLink ? baseLink->R()(2,2) : 1.0;
                debugLog << t << "," << isFallen << "," << recoveryState << "," << recoveryTimer << "," 
                         << up_z << "," << isProne << "," 
                         << (r_hip ? r_hip->q_target() : 0) << "," 
                         << (r_knee ? r_knee->q_target() : 0) << "," 
                         << (r_ankle ? r_ankle->q_target() : 0) << "," 
                         << (r_arm ? r_arm->q_target() : 0) << "\n";
            }
            
            return true;
        }
        
        Link* baseLink = body_->link("base_link");
        if(baseLink){
            Matrix3 R = baseLink->R();
            double up_z = R(2,2); // Dot product of local Z and World Z
            if(up_z < 0.6){ // Tilted more than ~53 degrees
                std::cout << "[jalan_lari] DETEKSI JATUH! (up_z=" << up_z << "). Memulai sekuens Bangkit (Recovery)!" << std::endl;
                isFallen = true;
                recoveryState = 0;
                recoveryTimer = 0.0;
                
                // Save current joint states for interpolation
                q_fall.resize(body_->numJoints(), 0.0);
                for(int i=0; i<body_->numJoints(); ++i){
                    q_fall[i] = body_->joint(i)->q();
                    body_->joint(i)->q_target() = q_fall[i]; // Freeze instantly
                }
                
                return true;
            }
        }

        if (firstTick) {
            startX = root_x ? root_x->q() : 0.0;
            startY = root_y ? root_y->q() : 0.0;
            startYaw = root_yaw ? root_yaw->q() : 0.0;

            double initialZ = STAND_ROOT_Z;
            if (USE_DIAM_AWAL) {
                initialZ = 0.8;
            }

            applyStandPose(false, startX, startY, initialZ, startYaw);

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
        bool isHolding = false;
        double holdX = freezeX;
        double holdY = freezeY;
        double holdYaw = freezeYaw;

        if (isActiveWindow(USE_DIAM_AWAL, cursor, T_DIAM_AWAL, localT)) {
            holdX = freezeX; holdY = freezeY; holdYaw = freezeYaw;
            
            // Animasi jatuh (drop)
            double dropStartZ = 0.8; // Mulai dari ketinggian 0.8 meter
            double currentZ = STAND_ROOT_Z;
            double dropDuration = 1.0; // Jatuh selama 1 detik
            
            if (localT < dropDuration) {
                double progress = localT / dropDuration;
                // ease-in ease-out smoothing
                double smooth = progress * progress * (3.0 - 2.0 * progress);
                currentZ = dropStartZ + (STAND_ROOT_Z - dropStartZ) * smooth;
            }
            
            setRoot(holdX, holdY, currentZ, holdYaw, false);
            isHolding = true;
            dispatched = true;
        }

        if (!dispatched && isActiveWindow(USE_JALAN_MAJU, cursor, T_JALAN_MAJU, localT)) {
            yAfterMaju = nodeJalanMaju(localT, T_JALAN_MAJU,
                                       freezeX, freezeY, freezeYaw, rootYSign, walkSpeed, rootRampDur);
            dispatched = true;
        } else if (USE_JALAN_MAJU) {
            yAfterMaju = freezeY + rootYSign * forwardRootY(T_JALAN_MAJU, walkSpeed, rootRampDur, T_JALAN_MAJU);
        } else {
            yAfterMaju = freezeY;
        }

        if (!dispatched && isActiveWindow(USE_DIAM_TRANSISI, cursor, T_DIAM_TRANSISI, localT)) {
            holdX = freezeX; holdY = yAfterMaju; holdYaw = freezeYaw;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }

        if (!dispatched && isActiveWindow(USE_LARI_MAJU, cursor, T_LARI_MAJU, localT)) {
            yAfterLari = nodeLariMaju(localT, T_LARI_MAJU,
                                      freezeX, yAfterMaju, freezeYaw, rootYSign, runSpeed, rootRampDur);
            dispatched = true;
        } else if (USE_LARI_MAJU) {
            yAfterLari = yAfterMaju + rootYSign * forwardRootY(T_LARI_MAJU, runSpeed, rootRampDur, T_LARI_MAJU);
        } else {
            yAfterLari = yAfterMaju;
        }

        if (!dispatched && isActiveWindow(USE_DIAM_AKHIR, cursor, T_DIAM_AKHIR, localT)) {
            holdX = freezeX; holdY = yAfterLari; holdYaw = freezeYaw;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }

        if (!dispatched) {
            holdX = freezeX; holdY = yAfterLari; holdYaw = freezeYaw;
            setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
        }

        if (isHolding) {
            // Kita biarkan setRoot di dalam blok state masing-masing mengatur tinggi Z
            // Jangan override posisinya di sini, agar animasi drop berfungsi!
            // lockRootOnFloor(true, holdX, holdY, holdYaw);
        } else if (floorMode && root_z) {
            // NO MAGIC ANCHOR
        }

        // --- ACTIVE BALANCER FOR SIM-TO-REAL ---
        if (body_->rootLink()) {
            Matrix3 R = body_->rootLink()->R();
            double current_pitch = std::asin(std::clamp(-R(2,0), -1.0, 1.0)); 
            double current_roll = std::atan2(R(2,1), R(2,2));

            static double last_pitch = current_pitch;
            static double last_roll = current_roll;
            double dt = io_->timeStep();
            double dpitch = (current_pitch - last_pitch) / dt;
            double droll = (current_roll - last_roll) / dt;
            last_pitch = current_pitch;
            last_roll = current_roll;

            // Simple PD gains
            double kp_pitch = 0.4;
            double kd_pitch = 0.02;
            double kp_roll = 0.3;
            double kd_roll = 0.01;

            double pitch_corr = kp_pitch * current_pitch + kd_pitch * dpitch;
            double roll_corr = kp_roll * current_roll + kd_roll * droll;

            // Cap the corrections to prevent physics explosions!
            pitch_corr = std::clamp(pitch_corr, -0.1, 0.1);
            roll_corr = std::clamp(roll_corr, -0.1, 0.1);

            // Apply corrections to ankles
            // R_ankle positive is toe UP -> pitch forward means need toe DOWN (negative)
            if (r_ankle) r_ankle->q_target() -= pitch_corr;
            // L_ankle negative is toe UP -> pitch forward means need toe DOWN (positive)
            if (l_ankle) l_ankle->q_target() += pitch_corr;

            if (r_ankle_roll) r_ankle_roll->q_target() -= roll_corr;
            if (l_ankle_roll) l_ankle_roll->q_target() += roll_corr;
        }

        body_->calcForwardKinematics();

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(jalan_lari)
