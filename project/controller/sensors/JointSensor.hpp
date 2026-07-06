#ifndef SENSORS_JOINT_SENSOR_HPP
#define SENSORS_JOINT_SENSOR_HPP

#include "SensorTypes.hpp"
#include "RobotState.hpp"
#include <cnoid/Body>
#include <cnoid/SimpleController>

namespace sensors {

/**
 * @brief Reads joint positions, velocities, and efforts.
 * Also reads root link position and orientation.
 */
class JointSensor : public ISensor {
public:
    bool initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) override;
    void update(RobotState& state) override;
    std::string getName() const override { return "JointSensor"; }

private:
    cnoid::Body* body_ = nullptr;
    cnoid::Link* root_ = nullptr;
};

} // namespace sensors
#endif // SENSORS_JOINT_SENSOR_HPP
