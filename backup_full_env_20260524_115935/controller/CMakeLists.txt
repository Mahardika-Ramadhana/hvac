cmake_minimum_required(VERSION 3.16)
project(RobotBesarControllers CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/choreonoid/cmake")
find_package(Choreonoid REQUIRED)

include("/usr/local/share/choreonoid/cmake/ext/ChoreonoidBodyBuildFunctions.cmake")

# ─────────────────────────────────────────────────────────────
#  Motion-planning library
#  Compile all RobotContext + per-motion node files together.
# ─────────────────────────────────────────────────────────────
set(MOTION_SOURCES
    RobotContext.cpp
    JalanDitempat.cpp
    JalanMaju.cpp
    Lompat.cpp
    JalanMundur.cpp
    HadapKanan90.cpp
    HadapKiri90.cpp
    PutarKanan360.cpp
    PutarKiri360.cpp
)

# Simple controllers
choreonoid_add_simple_controller(RobotBesarMinimumController
    RobotBesarMinimumController.cpp)

choreonoid_add_simple_controller(RobotBesarWalkController
    RobotBesarWalkController.cpp
    ${MOTION_SOURCES})
