#ifndef ROBOT_ESR_V2_HADAP_KIRI90_HPP
#define ROBOT_ESR_V2_HADAP_KIRI90_HPP

#include "RobotContext.hpp"

// Turn to face left (~90°) while marching in place.
void nodeHadapKiri90(RobotContext& robot,
                     double t,
                     double duration,
                     double freezeX,
                     double freezeY,
                      double startYaw);

#endif
