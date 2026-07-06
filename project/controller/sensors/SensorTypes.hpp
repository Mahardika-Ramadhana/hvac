#ifndef SENSORS_SENSOR_TYPES_HPP
#define SENSORS_SENSOR_TYPES_HPP

#include <cnoid/EigenTypes>
#include <string>

namespace cnoid { class SimpleControllerIO; class Body; }

namespace sensors {

/**
 * @brief Enum defining various sensor statuses.
 */
enum class SensorStatus {
    OK,
    NOT_AVAILABLE,
    INITIALIZATION_FAILED,
    TIMEOUT,
    INVALID_POINTER
};

/**
 * @brief Convert status to string for logging.
 */
inline std::string statusToString(SensorStatus status) {
    switch(status) {
        case SensorStatus::OK: return "OK";
        case SensorStatus::NOT_AVAILABLE: return "NOT_AVAILABLE";
        case SensorStatus::INITIALIZATION_FAILED: return "INITIALIZATION_FAILED";
        case SensorStatus::TIMEOUT: return "TIMEOUT";
        case SensorStatus::INVALID_POINTER: return "INVALID_POINTER";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Struct to hold IMU data.
 */
struct ImuData {
    SensorStatus status = SensorStatus::NOT_AVAILABLE;
    cnoid::Vector3 linearAcceleration = cnoid::Vector3::Zero();
    cnoid::Vector3 angularVelocity = cnoid::Vector3::Zero();
    cnoid::Vector3 rpy = cnoid::Vector3::Zero(); // Estimated roll, pitch, yaw
};

struct RobotState; // forward declaration

/**
 * @brief Interface for all sensors.
 */
class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool initialize(cnoid::SimpleControllerIO* io, cnoid::Body* body) = 0;
    virtual void update(RobotState& state) = 0;
    virtual std::string getName() const = 0;
};

} // namespace sensors

#endif // SENSORS_SENSOR_TYPES_HPP
