#ifndef ROBOT_ESR_V2_JALAN_MUNDUR_HPP
#define ROBOT_ESR_V2_JALAN_MUNDUR_HPP

#include "RobotContext.hpp"

// Walk backward starting from yStart (typically the y reached at the
// end of nodeJalanMaju).
double nodeJalanMundur(RobotContext& robot,
                       double t,
                       double duration,
                       double freezeX,
                       double yStart,
                       double startYaw,
                       double rootYSign,
                       double speed,
                       double rootRampDur);

#endif
