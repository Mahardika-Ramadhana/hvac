#include <cnoid/JointPath>
// =====================================================================
//  RobotBesarWalkController — master motion dispatcher for robot_esr_v2
//
//  Two control modes are supported, chosen by the option string set on
//  the SimpleControllerItem (or via the simulator's controller_options):
//      "pid"  / "torque"  / "forward_dyn"  → PID torque control suitable
//                                            for AISTSimulator dynamicsMode
//                                            = "Forward dynamics".
//      "forward_pos" / "kinematics" / ""    → JointDisplacement (high-gain
//                                            position) control suitable
//                                            for AISTSimulator dynamicsMode
//                                            = "Kinematics".
//
//  Each control tick:
//    1. Reset all joint targets to neutral pose.
//    2. Dispatch the active motion node (JalanMaju, HadapKanan90, …)
//       which writes q_target on the joints.
//    3. If PID mode is active, compute and apply a PID torque for every
//       joint (q_target → u).
//    4. Periodically dump joint states (q, dq, q_target, u) to console.
// =====================================================================
#include <cstddef>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include <cnoid/SimpleController>
#include <cnoid/BodyItem>

#include "RobotContext.hpp"
#include "BalanceMarkers.hpp"
#include "JalanDitempat.hpp"
#include "JalanMaju.hpp"
#include "JalanMundur.hpp"
#include "HadapKanan90.hpp"
#include "HadapKiri90.hpp"
#include "PutarKanan360.hpp"
#include "PutarKiri360.hpp"
#include "Lompat.hpp"
#include "sensors/SensorManager.hpp"

using namespace cnoid;

// ─────────────────────────────────────────────────────────────────
//  PID gain table.  Indices follow the URDF / RobotContext layout:
//    0..3   = virtual root joints (root_x, root_y, root_z, root_yaw)
//    4..15  = leg joints
//    16..17 = body_pitch, body_yaw
//    18..35 = arms (right then left)
//    36..37 = head
// ─────────────────────────────────────────────────────────────────
struct PIDGains {
    double Kp;
    double Ki;
    double Kd;
    double iLim;
    double uLim;   // max torque/force per joint
};

static PIDGains gainsFor(int i)
{
    // Gains are tuned for a relatively small robot (≈ 10–15 kg) where
    // most link inertias are O(1e-3) kg·m².  Use conservative values to
    // avoid runaway acceleration; the integral term picks up steady-
    // state error.
    //
    // Virtual root joints are position-controlled (JointDisplacement),
    // so their PID gains below are dormant unless someone forces them
    // to torque mode.
    if(i == 0 || i == 1){
        return { 1500.0,  30.0,  120.0, 0.4, 400.0 };
    }
    if(i == 2){
        return { 3000.0,  80.0,  200.0, 0.4, 800.0 };
    }
    if(i == 3){
        return {  800.0,  20.0,   80.0, 0.4, 300.0 };
    }

    // ── Leg pitch chain (hip / knee / ankle) ──
    if(i >= 4 && i <= 15){
        return { 55.0, 2.5, 2.5, 0.25, 45.0 };
    }

    // ── Body ──
    if(i == 16 || i == 17){
        return { 40.0, 1.5, 1.8, 0.25, 30.0 };
    }

    // ── Arms / hands ──
    if(i >= 18 && i <= 35){
        return { 25.0, 1.0, 1.0, 0.2, 15.0 };
    }

    // ── Head ──
    return { 20.0, 0.8, 0.8, 0.3, 12.0 };
}

class RobotBesarWalkController : public SimpleController
{
private:
    RobotContext robot;
    BalanceMarkers balanceMarkers;
    BodyItem* bodyItem_ = nullptr;
    SimpleControllerIO* io_ = nullptr;

    double t = 0.0;
    double dt = 0.0;

    std::unique_ptr<sensors::SensorManager> sensorManager_;

    double freezeX = 0.0;
    double freezeY = 0.0;
    double freezeYaw = 0.0;
    double yAfterMaju = 0.0;
    double yAfterMundur = 0.0;

    // PID state
    bool usePID = false;
    bool firstTick = true;
    std::vector<double> qPrev;
    std::vector<double> qErrInt;
    std::vector<double> qAtStart;     // joint q captured at sim start
    double rzAtStart = 0.0;           // root_z at sim start

    // Status print throttling
    int    tickCount = 0;
    int    printEvery = 500;     // 500 ticks * 0.001s = 0.5s
    bool   verbose = true;

    // ─────────────────────────────────────────────────────────
    //  ACTIVATE / DEACTIVATE MOTIONS
    // ─────────────────────────────────────────────────────────
    const bool USE_DIAM_AWAL       = true;
    const bool USE_JALAN_TEMPAT    = true;
    const bool USE_JALAN_MAJU      = true;
    const bool USE_JALAN_MUNDUR    = true;
    const bool USE_HADAP_KANAN90   = true;
    const bool USE_HADAP_KIRI90    = true;
    const bool USE_STOP_BEFORE_LOMPAT = true;
    const bool USE_LOMPAT          = true;
    const bool USE_PUTAR_KANAN360  = true;
    const bool USE_PUTAR_KIRI360   = true;
    const bool USE_DIAM_AKHIR      = true;

    // Floor-contact mode: after settle, root_z height follows physics
    const bool floorMode = true;

    // ─────────────────────────────────────────────────────────
    //  WINDOW DURATIONS  (seconds)
    // ─────────────────────────────────────────────────────────
    const double T_DIAM_AWAL       = 2.0;   // hold stand on floor (no rise ramp)
    const double T_JALAN_TEMPAT    = 3.0;
    const double T_JALAN_MAJU      = 15.0;
    const double T_JALAN_MUNDUR    = 5.0;
    const double T_HADAP_KANAN90   = 0.0;
    const double T_HADAP_KIRI90    = 0.0;
    const double T_STOP_BEFORE_LOMPAT = 0.0;
    const double T_LOMPAT          = 2.0;
    const double T_PUTAR_KANAN360  = 0.0;
    const double T_PUTAR_KIRI360   = 6.0;
    const double T_DIAM_AKHIR      = 4.0;   // recovery stand after jump

    // ─────────────────────────────────────────────────────────
    //  MOTION PARAMETERS (TUNED FOR RUNNING SHORT TRACK)
    // ─────────────────────────────────────────────────────────
    const double rootYSign   = -1.0;
    const double walkSpeed   = 0.25; // Kecepatan lari stabil
    const double cycleTime   = 0.5;  // Siklus lari yang stabil

public:
    bool configure(SimpleControllerConfig* config) override
    {
        bodyItem_ = dynamic_cast<BodyItem*>(config->bodyItem());
        return true;
    }

    bool start() override
    {
        if(bodyItem_){
            balanceMarkers.setup(bodyItem_);
        }
        return true;
    }

    void stop() override
    {
        balanceMarkers.clear();
    }

    bool initialize(SimpleControllerIO* io) override
    {
        io_ = io;
        dt = io->timeStep();
        t  = 0.0;

        // Decide control mode from option string.
        const std::string opt = io->optionString();
        usePID = (opt.find("pid")        != std::string::npos) ||
                 (opt.find("torque")     != std::string::npos) ||
                 (opt.find("forward_dyn")!= std::string::npos);

        std::cout << "=== RobotBesarWalkController :: MOTION DISPATCHER ==="
                  << std::endl;
        std::cout << "optionString = \"" << opt << "\"  →  mode = "
                  << (usePID ? "ForwardDynamicsPID (q_target + sim PID torque)"
                             : "POSITION (kinematics)") << std::endl;

        // Initialise RobotContext first (it defaults every joint to
        // JointDisplacement / position mode and enables output).
        const bool ok = robot.initialize(io);
        if(!ok) return false;

        // Start already standing on the floor (matches .cnoid initial pose).
        robot.applyStandPose(true);
        robot.body->calcForwardKinematics();

        // Recalculate foot orientations using actual stand pose (not q=0 default)
        {
            auto* base = robot.body->link("base_link");
            if(base && robot.r_ankle_roll && robot.l_ankle_roll) {
                robot.rFootRot0 = base->R().transpose() * robot.r_ankle_roll->R();
                robot.lFootRot0 = base->R().transpose() * robot.l_ankle_roll->R();
                cnoid::Vector3 rPos = robot.r_ankle_roll->p() - base->p();
                cnoid::Vector3 lPos = robot.l_ankle_roll->p() - base->p();
                std::cout << "[IK INIT] Right Foot Pos in base frame: " << rPos.transpose() << std::endl;
                std::cout << "[IK INIT] Left  Foot Pos in base frame: " << lPos.transpose() << std::endl;
                std::cout << "[IK INIT] rFootRot0 recalculated from stand pose." << std::endl;
            } else {
                std::cout << "[IK INIT] WARNING: base_link or ankle_roll not found!" << std::endl;
            }
        }

        // ── Set actuation mode + enable IO ──────────────────
        // ForwardDynamicsPID simulator applies high-gain PID internally
        // (ForwardDynamicsCBM) when controller_options = "pid".
        // All joints stay in JointDisplacement; we command q_target
        // trajectories and the simulator resolves contact/slip/fall.
        {
            const int inputFlags =
                (int)Link::JointDisplacement |
                (int)Link::JointVelocity |
                (int)Link::LinkContactState;
            std::cout << "Jumlah joint: " << robot.body->numJoints() << "\n";
            for(int i = 0; i < robot.body->numJoints(); ++i){
                Link* j = robot.body->joint(i);
                if(!j) {
                    std::cerr << "[WARNING] Joint " << i << " is null!" << std::endl;
                    continue;
                }
                std::cout << "Joint " << i << ": " << j->name() << " = " << j->q() << "\n";
                j->setActuationMode(Link::JointDisplacement);
                io_->enableInput(j, inputFlags);
                io_->enableOutput(j);
            }
            for(int i = 0; i < robot.body->numLinks(); ++i){
                Link* link = robot.body->link(i);
                if(!link) continue;
                io_->enableInput(link, (int)Link::LinkContactState);
            }
            std::cout << "[CTRL] contact sensing ON (all links/joints)"
                      << std::endl;
            std::cout << "[CTRL] floorMode=" << (floorMode ? "ON" : "OFF")
                      << "  spawn=STAND@0.81m  collision=ALL+self"
                      << "  ForwardDynamicsPID (simulator CBM PID)"
                      << std::endl;
        }

        const int N = robot.body->numJoints();
        qPrev.assign(N, 0.0);
        qErrInt.assign(N, 0.0);
        for(int i = 0; i < N; ++i){
            qPrev[i] = robot.body->joint(i)->q();
            robot.body->joint(i)->q_target() = robot.body->joint(i)->q();
        }

        freezeX = robot.root_x ? robot.root_x->q() : 0.0;
        freezeY = robot.root_y ? robot.root_y->q() : 0.0;
        freezeYaw = robot.root_yaw ? robot.root_yaw->q() : 0.0;
        std::cout << "freezeX = " << freezeX
                  << "  freezeY = " << freezeY
                  << "  freezeYaw = " << freezeYaw
                  << "  root_z (forced) = "
                  << (robot.root_z ? robot.root_z->q() : 0.0) << std::endl;

        sensors::SensorConfig sConfig;
        sConfig.enableLogging = true;
        sensorManager_ = std::make_unique<sensors::SensorManager>(sConfig);
        if(!sensorManager_->initialize(io_, robot)) {
            std::cerr << "[Warning] SensorManager initialization had missing sensors (likely IMU)." << std::endl;
        }

        return true;
    }

    // Return true when `t` is inside [cursor, cursor+duration).  Cursor
    // advances regardless of enabled state so disabled windows still
    // consume their nominal duration on the timeline.
    bool isActiveWindow(bool enabled,
                        double& cursor,
                        double duration,
                        double& localT)
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
        // Re-sync qPrev with the actual joint state on the very first
        // control tick.  The body's initialJointDisplacements are not
        // applied at initialize() time for URDF models with virtual
        // root joints, so we capture the real start state here and
        // also immediately set q_target = q to avoid a huge step
        // error that would otherwise blow up the integrator.
        if(firstTick){
            const int N = robot.body->numJoints();
            qAtStart.resize(N);
            // Re-apply stand on first sim tick (URDF may reset q before this).
            robot.applyStandPose(true);
            std::cout << "[CTRL] first tick: stand pose locked on floor"
                      << std::endl;
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                if(!j) continue;
                qPrev[i] = j->q();
                qAtStart[i] = j->q();
                j->q_target() = j->q();
                qErrInt[i] = 0.0;
            }
            rzAtStart = robot.root_z ? robot.root_z->q() : 0.81;
            std::cout << "  root_z = " << rzAtStart << " m (feet on floor)"
                      << std::endl;
            firstTick = false;
            return true;
        }

        t += dt;
        robot.resetAllTargets();

        double cursor = 0.0;
        double localT = 0.0;

        bool dispatched = false;
        bool isJumping = false;
        bool isHolding = false;
        double holdX = freezeX;
        double holdY = freezeY;
        double holdYaw = freezeYaw;

        // State-continuity yaw anchors.
        // Motion berikutnya memakai heading akhir motion sebelumnya.
        const double yawAfterKanan90  = USE_HADAP_KANAN90  ? freezeYaw - M_PI_2       : freezeYaw;
        const double yawAfterKiri90   = USE_HADAP_KIRI90   ? yawAfterKanan90 + M_PI_2 : yawAfterKanan90;
        const double yawAfterLompat   = yawAfterKiri90;
        const double yawAfterPutarKanan360 = USE_PUTAR_KANAN360 ? yawAfterLompat - 2.0 * M_PI : yawAfterLompat;
        const double yawAfterPutarKiri360  = USE_PUTAR_KIRI360  ? yawAfterPutarKanan360 + 2.0 * M_PI : yawAfterPutarKanan360;
        const double yawFinal = yawAfterPutarKiri360;

        if(isActiveWindow(USE_DIAM_AWAL, cursor, T_DIAM_AWAL, localT)){
            holdX = freezeX; holdY = freezeY; holdYaw = freezeYaw;
            robot.setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_TEMPAT, cursor, T_JALAN_TEMPAT, localT)){
            nodeJalanMaju(robot, localT, T_JALAN_TEMPAT, freezeX, freezeY, freezeYaw, 0.0, 2.0); // stepY = 0, cycle = 2.0s
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_MAJU, cursor, T_JALAN_MAJU, localT)){
            double ramp = std::min(localT / 2.0, 1.0); // Smooth acceleration
            yAfterMaju = nodeJalanMaju(robot, localT, T_JALAN_MAJU,
                                       freezeX, freezeY, freezeYaw,
                                       rootYSign * walkSpeed * ramp, cycleTime); // cycleTime = 0.4s (fast run)
            dispatched = true;
        } else if(USE_JALAN_MAJU){
            yAfterMaju = freezeY + (T_JALAN_MAJU / 2.0) * 0.10;
        } else {
            yAfterMaju = freezeY;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_MUNDUR, cursor, T_JALAN_MUNDUR, localT)){
            yAfterMundur = nodeJalanMaju(
                robot, localT, T_JALAN_MUNDUR,
                freezeX, yAfterMaju, freezeYaw, -0.10, 2.0); // stepY = -10cm, cycle = 2.0s
            dispatched = true;
        } else if(!dispatched){
            yAfterMundur = yAfterMaju;
        }
        if(!dispatched &&
           isActiveWindow(USE_HADAP_KANAN90, cursor, T_HADAP_KANAN90, localT)){
            nodeHadapKanan90(robot, localT, T_HADAP_KANAN90,
                             freezeX, yAfterMundur, freezeYaw);
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_HADAP_KIRI90, cursor, T_HADAP_KIRI90, localT)){
            nodeHadapKiri90(robot, localT, T_HADAP_KIRI90,
                            freezeX, yAfterMundur, yawAfterKanan90);
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_STOP_BEFORE_LOMPAT, cursor, T_STOP_BEFORE_LOMPAT, localT)){
            holdX = freezeX; holdY = yAfterMundur; holdYaw = yawAfterLompat;
            robot.setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_LOMPAT, cursor, T_LOMPAT, localT)){
            nodeLompat(robot, localT, T_LOMPAT, freezeX, yAfterMundur, yawAfterLompat);
            isJumping = true;
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_PUTAR_KANAN360, cursor, T_PUTAR_KANAN360, localT)){
            nodePutarKanan360(robot, localT, T_PUTAR_KANAN360,
                              freezeX, yAfterMundur, yawAfterLompat);
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_PUTAR_KIRI360, cursor, T_PUTAR_KIRI360, localT)){
            nodePutarKiri360(robot, localT, T_PUTAR_KIRI360,
                             freezeX, yAfterMundur, yawAfterPutarKanan360);
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_DIAM_AKHIR, cursor, T_DIAM_AKHIR, localT)){
            holdX = freezeX; holdY = yAfterMundur; holdYaw = yawFinal;
            robot.setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
            dispatched = true;
        }
        if(!dispatched){
            holdX = freezeX; holdY = yAfterMundur; holdYaw = yawFinal;
            robot.setRoot(holdX, holdY, 0.0, holdYaw, true);
            isHolding = true;
        }

        // Hold phases: paksa pose berdiri penuh di floor (q + q_target).
        if(isHolding){
            // setActualQ=false: only update q_target, let physics handle actual q
            robot.applyStandPose(false, holdX, holdY, holdYaw);
            robot.lockRootOnFloor(false, holdX, holdY, holdYaw);
        } else if(floorMode && !isJumping && robot.root_z){
            robot.root_z->q_target() = RobotContext::STAND_ROOT_Z;
        }

        robot.body->calcForwardKinematics();
        const BalanceMetrics bm = balanceMarkers.update(robot.body);

        // ─────────────────────────────────────────────────────
        //  PID state for diagnostics (sim applies actual torque)
        // ─────────────────────────────────────────────────────
        {
            const int N = robot.body->numJoints();
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                if(!j) continue;
                const double q  = j->q();
                const double qt = j->q_target();
                qErrInt[i] += (qt - q) * dt;
                const PIDGains g = gainsFor(i);
                if(qErrInt[i] >  g.iLim) qErrInt[i] =  g.iLim;
                if(qErrInt[i] < -g.iLim) qErrInt[i] = -g.iLim;
            }
        }

        // ─────────────────────────────────────────────────────
        //  Advance qPrev for next tick's derivative
        // ─────────────────────────────────────────────────────
        {
            const int N = robot.body->numJoints();
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                if(j) qPrev[i] = j->q();
            }
        }

        // ─────────────────────────────────────────────────────
        // ─────────────────────────────────────────────────────
        ++tickCount;

        if (sensorManager_) {
            sensorManager_->update(t);
            if(verbose && (tickCount % printEvery) == 0){
                sensorManager_->printDebugLog();
            }
        }

        if(verbose && (tickCount % printEvery) == 0){
            printJointStates(bm);
        }

        return true;
    }

private:
    // ─────────────────────────────────────────────────────
    //  Theoretical PID torque – computed for diagnostic display.
    //  Not applied to the joint; the actual position tracking is
    //  done by Choreonoid's internal high-gain PD.  These numbers
    //  show what an explicit Kp·err + Ki·∫err + Kd·erṙ controller
    //  WOULD produce, useful for tuning.
    // ─────────────────────────────────────────────────────
    double theoreticalTorque(int i, double q, double qt, double dq) const
    {
        const PIDGains g = gainsFor(i);
        const double err = qt - q;
        double u = g.Kp * err
                 + g.Ki * qErrInt[i]
                 + g.Kd * (0.0 - dq);
        if(u >  g.uLim) u =  g.uLim;
        if(u < -g.uLim) u = -g.uLim;
        return u;
    }

    void printJointStates(const BalanceMetrics& bm) const
    {
        const int N = robot.body->numJoints();

        auto writeTo = [&](std::ostream& os){
            os << "\n════════  t = " << std::fixed << std::setprecision(3)
               << t << " s   ForwardDynamicsPID  ════════\n";
            os << "CoM  [" << bm.com.transpose() << "]\n";
            os << "CoG  [" << bm.cog.transpose() << "]  (same as CoM in uniform g)\n";
            if(bm.copValid){
                os << "CoP  [" << bm.cop.transpose() << "]"
                   << "  |F|=" << bm.totalContactForce
                   << " N  contacts=" << bm.numContacts << "\n";
            } else {
                os << "CoP  [no foot contact]"
                   << "  |F|=" << bm.totalContactForce
                   << " N  contacts=" << bm.numContacts << "\n";
            }
            os << "CoM_proj [" << bm.comXY.transpose() << "]\n";
            os << std::left << std::setw(4) << "id"
               << std::setw(22) << "name"
               << std::right
               << std::setw(10) << "q"
               << std::setw(10) << "q_tgt"
               << std::setw(10) << "err"
               << std::setw(10) << "dq"
               << std::setw(12) << "u_sim" << "\n";

            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                const double q   = j->q();
                const double qt  = j->q_target();
                const double err = qt - q;
                const double dq  = (i < (int)qPrev.size())
                                     ? (q - qPrev[i]) / dt : 0.0;
                const double u   = j->u();

                os << std::left  << std::setw(4) << i
                   << std::setw(22) << j->name()
                   << std::right << std::fixed << std::setprecision(3)
                   << std::setw(10) << q
                   << std::setw(10) << qt
                   << std::setw(10) << err
                   << std::setw(10) << dq
                   << std::setw(12) << u << "\n";
            }
            os.flush();
        };

        if(io_) writeTo(io_->os());
        writeTo(std::cout);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotBesarWalkController)
