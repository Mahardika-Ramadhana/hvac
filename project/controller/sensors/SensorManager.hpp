#ifndef SENSORS_SENSOR_MANAGER_HPP
#define SENSORS_SENSOR_MANAGER_HPP

#include "SensorConfig.hpp"
#include "RobotState.hpp"
#include "SensorTypes.hpp"
#include "../RobotContext.hpp"
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <memory>
#include <vector>

namespace sensors {

/**
 * @brief Manages the lifecycle and updating of all sensors.
 */
class SensorManager {
public:
    SensorManager(const SensorConfig& config = SensorConfig());
    ~SensorManager();

    /**
     * @brief Initialize all configured sensors.
     * @param io Pointer to the controller IO.
     * @param ctx The robot context providing necessary links.
     * @return true if all enabled sensors initialized successfully.
     */
    bool initialize(cnoid::SimpleControllerIO* io, const RobotContext& ctx);

    /**
     * @brief Update the internal robot state from all sensors.
     * @param timestamp Current simulation time.
     */
    void update(double timestamp);

    /**
     * @brief Get the latest RobotState.
     */
    const RobotState& getState() const { return state_; }

    /**
     * @brief Print a debug log if logging is enabled.
     */
    void printDebugLog() const;

private:
    SensorConfig config_;
    cnoid::SimpleControllerIO* io_ = nullptr;
    RobotState state_;
    
    std::vector<std::unique_ptr<ISensor>> sensors_;
};

} // namespace sensors
#endif // SENSORS_SENSOR_MANAGER_HPP
