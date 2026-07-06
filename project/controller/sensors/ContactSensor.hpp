#ifndef SENSORS_CONTACT_SENSOR_HPP
#define SENSORS_CONTACT_SENSOR_HPP

#include "SensorTypes.hpp"
#include "RobotState.hpp"
#include <cnoid/Body>
#include <cnoid/SimpleController>

namespace sensors {

/**
 * @brief Reads foot contact states based on LinkContactState.
 */
class ContactSensor : public ISensor {
public:
    /**
     * @brief Constructor requiring left and right foot links.
     */
    ContactSensor(cnoid::Link* leftFoot, cnoid::Link* rightFoot);

    bool initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) override;
    void update(RobotState& state) override;
    std::string getName() const override { return "ContactSensor"; }

private:
    cnoid::SimpleControllerIO* io_ = nullptr;
    cnoid::Link* leftFoot_ = nullptr;
    cnoid::Link* rightFoot_ = nullptr;
};

} // namespace sensors
#endif // SENSORS_CONTACT_SENSOR_HPP
