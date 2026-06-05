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

#include "RobotContext.hpp"
#include "JalanDitempat.hpp"
#include "JalanMaju.hpp"
#include "JalanMundur.hpp"
#include "HadapKanan90.hpp"
#include "HadapKiri90.hpp"
#include "PutarKanan360.hpp"
#include "PutarKiri360.hpp"
#include "Lompat.hpp"

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
        return { 90.0, 4.0, 3.0, 0.3, 60.0 };
    }

    // ── Body ──
    if(i == 16 || i == 17){
        return { 60.0, 2.5, 2.0, 0.3, 40.0 };
    }

    // ── Arms / hands ──
    if(i >= 18 && i <= 35){
        return { 35.0, 1.5, 1.2, 0.3, 20.0 };
    }

    // ── Head ──
    return { 20.0, 0.8, 0.8, 0.3, 12.0 };
}

class RobotBesarWalkController : public SimpleController
{
private:
    RobotContext robot;
    SimpleControllerIO* io_ = nullptr;

    double t = 0.0;
    double dt = 0.0;

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
    const bool USE_PUTAR_KANAN360  = false;
    const bool USE_PUTAR_KIRI360   = false;
    const bool USE_DIAM_AKHIR      = true;

    // Floor-contact mode: after settle, root_z height follows physics
    const bool floorMode = true;

    // ─────────────────────────────────────────────────────────
    //  WINDOW DURATIONS  (seconds)
    // ─────────────────────────────────────────────────────────
    const double T_DIAM_AWAL       = 4.0;
    const double T_JALAN_TEMPAT    = 10.0;
    const double T_JALAN_MAJU      = 9.0;
    const double T_JALAN_MUNDUR    = 6.0;
    const double T_HADAP_KANAN90   = 4.0;
    const double T_HADAP_KIRI90    = 4.0;
    const double T_STOP_BEFORE_LOMPAT = 2.0;
    const double T_LOMPAT          = 1.6;
    const double T_PUTAR_KANAN360  = 6.0;
    const double T_PUTAR_KIRI360   = 6.0;
    const double T_DIAM_AKHIR      = 2.0;

    // ─────────────────────────────────────────────────────────
    //  MOTION PARAMETERS
    // ─────────────────────────────────────────────────────────
    const double rootYSign   = -1.0;
    const double walkSpeed   = 0.05;
    const double rootRampDur = 1.2;

public:
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
                  << (usePID ? "HYBRID  root=POSITION  others=PID-TORQUE"
                             : "POSITION (kinematics)") << std::endl;

        // Initialise RobotContext first (it defaults every joint to
        // JointDisplacement / position mode and enables output).
        const bool ok = robot.initialize(io);
        if(!ok) return false;

        // ── Force the robot to start in a known stand pose ───
        // Choreonoid does not always apply BodyItem.initialJointDisplacements
        // to the simulator body before the first control tick (especially
        // for URDF-loaded models with virtual root joints), so we set
        // q explicitly here.  This avoids the body starting at z = 0.
        if(robot.root_z) robot.root_z->q() = 0.81;
        if(robot.r_hip)  robot.r_hip->q()  = -0.04;
        if(robot.r_knee) robot.r_knee->q() =  0.04;
        if(robot.r_ankle)robot.r_ankle->q()=  0.08;
        if(robot.l_hip)  robot.l_hip->q()  =  0.04;
        if(robot.l_knee) robot.l_knee->q() = -0.04;
        if(robot.l_ankle)robot.l_ankle->q()= -0.08;
        robot.body->calcForwardKinematics();

        // ── Set actuation mode + enable IO ──────────────────
        // We keep every joint in JointDisplacement (high-gain position)
        // mode.  Even with the simulator in dynamicsMode = Forward
        // Dynamics, this lets Choreonoid's internal PD generate the
        // torques required to track q_target.  The controller therefore
        // owns the high-level position trajectory while the simulator
        // resolves the physics.  We still LOG the simulator's torque
        // output (joint->u()) so the user can monitor what forces are
        // being applied.
        {
            const int N = robot.body->numJoints();
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                j->setActuationMode(Link::JointDisplacement);
                io_->enableInput(j);
                io_->enableOutput(j);
            }
        std::cout << "[CTRL] floorMode=" << (floorMode ? "ON" : "OFF")
                  << "  all joints = JointDisplacement (Forward Dynamics)"
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
            std::cout << "[CTRL] first tick joint state captured:" << std::endl;
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                qPrev[i] = j->q();
                qAtStart[i] = j->q();
                j->q_target() = j->q();        // no initial step error
                qErrInt[i] = 0.0;
            }
            rzAtStart = robot.root_z ? robot.root_z->q() : 0.0;
            std::cout << "  root_z initial = " << rzAtStart
                      << " (will ramp to 0.81 during DIAM_AWAL)" << std::endl;
            firstTick = false;
            // skip the rest of this tick so q_target is sent down as
            // matching values to the sim before any motion node runs.
            return true;
        }

        t += dt;
        robot.resetAllTargets();

        double cursor = 0.0;
        double localT = 0.0;

        bool dispatched = false;
        bool isJumping = false;

        // State-continuity yaw anchors.
        // Motion berikutnya memakai heading akhir motion sebelumnya.
        const double yawAfterKanan90  = USE_HADAP_KANAN90  ? freezeYaw - M_PI_2       : freezeYaw;
        const double yawAfterKiri90   = USE_HADAP_KIRI90   ? yawAfterKanan90 + M_PI_2 : yawAfterKanan90;
        const double yawAfterLompat   = yawAfterKiri90;
        const double yawAfterPutarKanan360 = USE_PUTAR_KANAN360 ? yawAfterLompat - 2.0 * M_PI : yawAfterLompat;
        const double yawAfterPutarKiri360  = USE_PUTAR_KIRI360  ? yawAfterPutarKanan360 + 2.0 * M_PI : yawAfterPutarKanan360;
        const double yawFinal = yawAfterPutarKiri360;

        if(isActiveWindow(USE_DIAM_AWAL, cursor, T_DIAM_AWAL, localT)){
            // Smoothly ramp every joint from its true start position
            // to the stand pose during the DIAM_AWAL window.  This
            // avoids the simulator seeing a huge initial step error.
            const double s = RobotContext::smoothstep(localT / T_DIAM_AWAL);
            const int N = robot.body->numJoints();
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                double qFinal;
                if(i == 2){
                    qFinal = 0.81;                       // root_z
                } else if(i == 3){
                    qFinal = freezeYaw;                  // root_yaw
                } else if(i < 4){
                    qFinal = 0.0;                        // root_x/y
                } else {
                    qFinal = robot.q0of(j);              // stand pose biases
                }
                j->q_target() = qAtStart[i] + (qFinal - qAtStart[i]) * s;
            }
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_TEMPAT, cursor, T_JALAN_TEMPAT, localT)){
            nodeJalanDitempat(robot, localT, freezeX, freezeY, freezeYaw);
            dispatched = true;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_MAJU, cursor, T_JALAN_MAJU, localT)){
            yAfterMaju = nodeJalanMaju(
                robot, localT, T_JALAN_MAJU,
                freezeX, freezeY, freezeYaw, rootYSign, walkSpeed, rootRampDur);
            dispatched = true;
        } else if(USE_JALAN_MAJU){
            yAfterMaju = freezeY + rootYSign *
                         RobotContext::forwardRootY(T_JALAN_MAJU, walkSpeed,
                                                    rootRampDur, T_JALAN_MAJU);
        } else {
            yAfterMaju = freezeY;
        }
        if(!dispatched &&
           isActiveWindow(USE_JALAN_MUNDUR, cursor, T_JALAN_MUNDUR, localT)){
            yAfterMundur = nodeJalanMundur(
                robot, localT, T_JALAN_MUNDUR,
                freezeX, yAfterMaju, freezeYaw, rootYSign, walkSpeed, rootRampDur);
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
            // Berhenti dulu sebelum lompat.
            // Tetap di posisi dan heading terakhir setelah hadap kiri.
            robot.setRoot(freezeX, yAfterMundur, 0.0, yawAfterLompat);
            if(robot.root_yaw){ robot.root_yaw->q_target() = yawAfterLompat; }
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
            robot.setRoot(freezeX, yAfterMundur, 0.0, yawFinal);
            if(robot.root_yaw){ robot.root_yaw->q_target() = yawFinal; }
            dispatched = true;
        }
        if(!dispatched){
            robot.setRoot(freezeX, yAfterMundur, 0.0, yawFinal);
            if(robot.root_yaw){ robot.root_yaw->q_target() = yawFinal; }
        }

        // Floor contact: after settle phase, do not lock virtual root_z.
        // Feet + forward dynamics determine body height; locking z at
        // 0.81 m fights the contact solver and causes instability.
        if(floorMode && !isJumping && t >= T_DIAM_AWAL && robot.root_z){
            robot.root_z->q_target() = robot.root_z->q();
        }

        // ─────────────────────────────────────────────────────
        //  Update PID state (qPrev for dq, integral term for u_theo)
        // ─────────────────────────────────────────────────────
        {
            const int N = robot.body->numJoints();
            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                const double q  = j->q();
                const double qt = j->q_target();
                qErrInt[i] += (qt - q) * dt;
                const PIDGains g = gainsFor(i);
                if(qErrInt[i] >  g.iLim) qErrInt[i] =  g.iLim;
                if(qErrInt[i] < -g.iLim) qErrInt[i] = -g.iLim;
                qPrev[i] = q;
            }
        }

        // ─────────────────────────────────────────────────────
        //  Periodic status dump – joint states & torques
        // ─────────────────────────────────────────────────────
        ++tickCount;
        if(verbose && (tickCount % printEvery) == 0){
            printJointStates();
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

    void printJointStates() const
    {
        const int N = robot.body->numJoints();

        auto writeTo = [&](std::ostream& os){
            os << "\n════════  t = " << std::fixed << std::setprecision(3)
               << t << " s   mode = FORWARD-DYN  (target = q_tgt, "
                   "u_theo = theoretical PID torque)  ════════\n";
            os << std::left << std::setw(4) << "id"
               << std::setw(22) << "name"
               << std::right
               << std::setw(10) << "q"
               << std::setw(10) << "q_tgt"
               << std::setw(10) << "err"
               << std::setw(10) << "dq"
               << std::setw(12) << "u_theo" << "\n";

            for(int i = 0; i < N; ++i){
                Link* j = robot.body->joint(i);
                const double q   = j->q();
                const double qt  = j->q_target();
                const double err = qt - q;
                const double dq  = (i < (int)qPrev.size())
                                     ? (q - qPrev[i]) / dt : 0.0;
                const double u   = theoreticalTorque(i, q, qt, dq);

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
