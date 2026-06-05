#ifndef ROBOT_ESR_V2_HADAP_KANAN90_HPP
#define ROBOT_ESR_V2_HADAP_KANAN90_HPP

#include "RobotContext.hpp"

// Turn to face right (~90°) while marching in place.
void nodeHadapKanan90(RobotContext& robot,
                      double t,
                      double duration,
                      double freezeX,
                      double freezeY,
                      double startYaw);

#endif
