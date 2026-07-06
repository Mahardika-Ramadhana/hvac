#ifndef SENSORS_IMU_SENSOR_HPP
#define SENSORS_IMU_SENSOR_HPP

#include "SensorTypes.hpp"
#include "RobotState.hpp"
#include <cnoid/Body>
#include <cnoid/SimpleController>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>

namespace sensors {

/**
 * @brief Reads IMU data (Acceleration and RateGyro).
 * If sensors are missing in URDF, sets state to NOT_AVAILABLE without crashing.
 */
class ImuSensor : public ISensor {
public:
    bool initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) override;
    void update(RobotState& state) override;
    std::string getName() const override { return "ImuSensor"; }

private:
    cnoid::AccelerationSensor* accelSensor_ = nullptr;
    cnoid::RateGyroSensor* gyroSensor_ = nullptr;
};

} // namespace sensors
#endif // SENSORS_IMU_SENSOR_HPP
