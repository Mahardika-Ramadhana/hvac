cmake_minimum_required(VERSION 3.16)
project(RobotBesarControllers CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_PREFIX_PATH "/home/dika/choreonoid/build/cmake")
find_package(Choreonoid REQUIRED)

include("/home/dika/choreonoid/src/Body/ChoreonoidBodyBuildFunctions.cmake")

# ─────────────────────────────────────────────────────────────
#  Motion-planning library
#  Compile all RobotContext + per-motion node files together.
# ─────────────────────────────────────────────────────────────
set(MOTION_SOURCES
    RobotContext.cpp
    BalanceMarkers.cpp
    JalanDitempat.cpp
    JalanMaju.cpp
    Lompat.cpp
    JalanMundur.cpp
    HadapKanan90.cpp
    HadapKiri90.cpp
    PutarKanan360.cpp
    PutarKiri360.cpp
    sensors/SensorManager.cpp
    sensors/JointSensor.cpp
    sensors/ContactSensor.cpp
    sensors/ImuSensor.cpp
)

# Simple controllers
choreonoid_add_simple_controller(RobotBesarMinimumController
    RobotBesarMinimumController.cpp)

choreonoid_add_simple_controller(RobotBesarWalkController
    RobotBesarWalkController.cpp
    ${MOTION_SOURCES})

choreonoid_add_simple_controller(jalan_lari jalan_lari.cpp)

target_link_libraries(RobotBesarWalkController Choreonoid::CnoidBodyPlugin)
include_directories(
  /home/dika/choreonoid
  /home/dika/choreonoid/src/BodyPlugin
  /home/dika/choreonoid/src/Body
  /home/dika/choreonoid/src/Util
  /home/dika/choreonoid/src/Base)

add_executable(test_ik test_ik.cpp)
target_link_libraries(test_ik Choreonoid::CnoidBody)
