#include "SensorManager.hpp"
#include "JointSensor.hpp"
#include "ContactSensor.hpp"
#include "ImuSensor.hpp"
#include <iostream>
#include <iomanip>

namespace sensors {

SensorManager::SensorManager(const SensorConfig& config) : config_(config) {
}

SensorManager::~SensorManager() {
}

bool SensorManager::initialize(cnoid::SimpleControllerIO* io, const RobotContext& ctx) {
    if (!io || !ctx.body) {
        std::cerr << "[SensorManager] ERROR: Invalid IO or Body pointer!" << std::endl;
        return false;
    }
    io_ = io;

    if (config_.enableJointSensor) {
        sensors_.push_back(std::make_unique<JointSensor>());
    }
    
    if (config_.enableContactSensor) {
        sensors_.push_back(std::make_unique<ContactSensor>(ctx.l_ankle_roll, ctx.r_ankle_roll));
    }
    
    if (config_.enableImuSensor) {
        sensors_.push_back(std::make_unique<ImuSensor>());
    }

    bool allSuccess = true;
    for (auto& s : sensors_) {
        if (!s->initialize(io, ctx.body)) {
            std::cerr << "[SensorManager] WARNING: Failed to initialize " << s->getName() << std::endl;
            allSuccess = false; 
        }
    }

    return allSuccess;
}

void SensorManager::update(double timestamp) {
    state_.timestamp = timestamp;
    state_.isValid = true;

    for (auto& s : sensors_) {
        s->update(state_);
    }
}

void SensorManager::printDebugLog() const {
    if (!config_.enableLogging || !state_.isValid) return;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[SensorManager] Time: " << state_.timestamp 
              << " | H: " << state_.rootPosition.z()
              << " | RPY: [" << state_.rootOrientation.x() << ", " 
                             << state_.rootOrientation.y() << ", "
                             << state_.rootOrientation.z() << "]"
              << " | LContact: " << (state_.leftFootContact ? "1" : "0")
              << " | RContact: " << (state_.rightFootContact ? "1" : "0")
              << " | Joints: " << state_.jointPosition.size()
              << " | IMU: " << statusToString(state_.imuData.status)
              << std::endl;
}

} // namespace sensors
