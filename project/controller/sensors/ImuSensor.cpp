#include "ImuSensor.hpp"
#include <iostream>

namespace sensors {

bool ImuSensor::initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) {
    if (!io || !body) return false;

    auto accels = body->devices<cnoid::AccelerationSensor>();
    auto gyros = body->devices<cnoid::RateGyroSensor>();

    if (!accels.empty()) {
        accelSensor_ = accels.front();
        io->enableInput(accelSensor_);
    } else {
        std::cerr << "[ImuSensor] INFO: AccelerationSensor NOT AVAILABLE in URDF." << std::endl;
    }

    if (!gyros.empty()) {
        gyroSensor_ = gyros.front();
        io->enableInput(gyroSensor_);
    } else {
        std::cerr << "[ImuSensor] INFO: RateGyroSensor NOT AVAILABLE in URDF." << std::endl;
    }

    // We return true even if IMU is missing, to allow the framework to run safely
    return true;
}

void ImuSensor::update(RobotState& state) {
    if (!accelSensor_ && !gyroSensor_) {
        state.imuData.status = SensorStatus::NOT_AVAILABLE;
        return;
    }

    state.imuData.status = SensorStatus::OK;

    if (accelSensor_) {
        state.imuData.linearAcceleration = accelSensor_->dv();
    }
    if (gyroSensor_) {
        state.imuData.angularVelocity = gyroSensor_->w();
    }
}

} // namespace sensors
