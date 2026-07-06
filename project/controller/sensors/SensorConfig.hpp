#ifndef SENSORS_SENSOR_CONFIG_HPP
#define SENSORS_SENSOR_CONFIG_HPP

namespace sensors {

/**
 * @brief Configuration parameters for the Sensor Framework.
 */
struct SensorConfig {
    bool enableLogging = false;       ///< Set to true to enable debug logging
    bool enableJointSensor = true;    ///< Enable/disable joint reading
    bool enableContactSensor = true;  ///< Enable/disable contact reading
    bool enableImuSensor = true;      ///< Enable/disable IMU reading
    int debugLevel = 1;               ///< Logging verbosity level (1=basic, 2=detailed)
};

} // namespace sensors
#endif // SENSORS_SENSOR_CONFIG_HPP
