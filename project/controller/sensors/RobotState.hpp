#ifndef SENSORS_ROBOT_STATE_HPP
#define SENSORS_ROBOT_STATE_HPP

#include <vector>
#include <cnoid/EigenTypes>
#include "SensorTypes.hpp"

namespace sensors {

/**
 * @brief Data container representing the full state of the robot at a given tick.
 * This class has no control logic, only pure data.
 */
struct RobotState {
    double timestamp = 0.0;     ///< Current simulation time
    bool isValid = false;       ///< True if state is successfully populated

    // Joint states
    std::vector<double> jointPosition;
    std::vector<double> jointVelocity;
    std::vector<double> jointEffort;

    // Root states
    cnoid::Vector3 rootPosition = cnoid::Vector3::Zero();
    cnoid::Vector3 rootOrientation = cnoid::Vector3::Zero(); // RPY
    cnoid::Vector3 rootLinearVelocity = cnoid::Vector3::Zero();
    cnoid::Vector3 rootAngularVelocity = cnoid::Vector3::Zero();

    // Contact states
    bool leftFootContact = false;
    bool rightFootContact = false;

    // IMU states
    ImuData imuData;
};

} // namespace sensors

#endif // SENSORS_ROBOT_STATE_HPP
