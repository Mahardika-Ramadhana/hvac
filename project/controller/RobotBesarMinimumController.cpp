/**
 * PD posture hold controller for robot_esr_v2 (34 joints).
 */

#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <vector>

using namespace cnoid;

namespace {

constexpr int NUM_JOINTS = 34;

double pgainForJoint(int index)
{
    if(index < 12){
        return 2500.0;
    }
    if(index < 14){
        return 1800.0;
    }
    return 1200.0;
}

double dgainForJoint(int index)
{
    (void)index;
    return 100.0;
}

}

class RobotBesarMinimumController : public SimpleController
{
    Body* ioBody;
    double dt;
    std::vector<double> qref;
    std::vector<double> qold;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();

        qref.clear();
        qold.clear();
        for(int i = 0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JointTorque);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;

        if(ioBody->numJoints() != NUM_JOINTS){
            io->os() << "RobotBesarMinimumController: expected "
                     << NUM_JOINTS << " joints, got "
                     << ioBody->numJoints() << std::endl;
            return false;
        }

        io->os() << "RobotBesarMinimumController: holding initial pose." << std::endl;
        return true;
    }

    virtual bool control() override
    {
        for(int i = 0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double u = (qref[i] - q) * pgainForJoint(i) + (0.0 - dq) * dgainForJoint(i);
            qold[i] = q;
            joint->u() = u;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotBesarMinimumController)
