#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <vector>
#include <cmath>

using namespace cnoid;

// Joint indices for robot_esr_v2
// Left leg:  hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll
// Right leg: hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll
const char* JOINT_NAMES[] = {
    // Left leg
    "left_knee_yaw_1",
    "left_knee_roll_1",
    "left_knee_pitch_1",
    "left_knee_pitch_3",
    "left_knee_pitch_5",
    "left_knee_roll_5",
    // Right leg
    "right_knee_yaw_2",
    "right_knee_roll_2",
    "right_knee_pitch_2",
    "right_knee_pitch_4",
    "right_knee_pitch_6",
    "right_knee_roll_6",
    // Torso
    "body_pitch",
    "body_yaw",
};
const int NUM_JOINTS = 14;

// Standing pose target angles (radians)
// [left: hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
// [right: same order]
// [torso: body_pitch, body_yaw]
const double STAND_POSE[] = {
    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // left leg
    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // right leg
    0.0,  0.0,                             // torso
};

// PD gains
const double KP = 20.0;
const double KD = 2.0;

class EsrController : public SimpleController {
    std::vector<Link*> joints;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override {
        dt = io->timeStep();
        joints.clear();

        for (int i = 0; i < NUM_JOINTS; i++) {
            Link* joint = io->body()->link(JOINT_NAMES[i]);
            if (!joint) {
                io->os() << "[esr_controller] joint not found: " << JOINT_NAMES[i] << std::endl;
                return false;
            }
            joint->setActuationMode(Link::JointTorque);
            io->enableIO(joint);
            joints.push_back(joint);
        }

        io->os() << "[esr_controller] initialized OK, " << NUM_JOINTS << " joints" << std::endl;
        return true;
    }

    virtual bool control() override {
        for (int i = 0; i < (int)joints.size(); i++) {
            double q    = joints[i]->q();
            double dq   = joints[i]->dq();
            double qref = STAND_POSE[i];
            joints[i]->u() = KP * (qref - q) - KD * dq;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(EsrController)
