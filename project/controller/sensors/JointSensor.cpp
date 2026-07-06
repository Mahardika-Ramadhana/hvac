#include "JointSensor.hpp"
#include <cnoid/EigenUtil>
#include <iostream>

namespace sensors {

bool JointSensor::initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) {
    if (!body || !io) {
        std::cerr << "[JointSensor] ERROR: Invalid pointer passed to initialize()." << std::endl;
        return false;
    }
    body_ = body;
    root_ = body_->rootLink();
    
    if (root_) {
        io->enableInput(root_, cnoid::Link::LinkPosition);
    }
    
    return true;
}

void JointSensor::update(RobotState& state) {
    if (!body_) return;

    const int numJoints = body_->numJoints();
    state.jointPosition.resize(numJoints, 0.0);
    state.jointVelocity.resize(numJoints, 0.0);
    state.jointEffort.resize(numJoints, 0.0);

    for (int i = 0; i < numJoints; ++i) {
        cnoid::Link* j = body_->joint(i);
        if (j) {
            state.jointPosition[i] = j->q();
            state.jointVelocity[i] = j->dq();
            state.jointEffort[i] = j->u();
        }
    }

    if (root_) {
        state.rootPosition = root_->translation();
        state.rootOrientation = cnoid::rpyFromRot(root_->rotation());
        state.rootLinearVelocity = root_->v();
        state.rootAngularVelocity = root_->w();
    }
}

} // namespace sensors
