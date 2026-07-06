#include "ContactSensor.hpp"
#include <iostream>

namespace sensors {

ContactSensor::ContactSensor(cnoid::Link* leftFoot, cnoid::Link* rightFoot)
    : leftFoot_(leftFoot), rightFoot_(rightFoot)
{
}

bool ContactSensor::initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) {
    if (!io || !body) return false;
    io_ = io;
    
    if (!leftFoot_ || !rightFoot_) {
        std::cerr << "[ContactSensor] WARNING: Foot link is null!" << std::endl;
        return false;
    }

    io->enableInput(leftFoot_, cnoid::Link::LinkContactState);
    io->enableInput(rightFoot_, cnoid::Link::LinkContactState);

    return true;
}

void ContactSensor::update(RobotState& state) {
    state.leftFootContact = false;
    state.rightFootContact = false;

    if (!leftFoot_ || !rightFoot_) return;

    if (leftFoot_->contactPoints().size() > 0) {
        state.leftFootContact = true;
    } else {
        if (leftFoot_->translation().z() < 0.04) {
            state.leftFootContact = true;
        }
    }

    if (rightFoot_->contactPoints().size() > 0) {
        state.rightFootContact = true;
    } else {
        if (rightFoot_->translation().z() < 0.04) {
            state.rightFootContact = true;
        }
    }
}

} // namespace sensors
