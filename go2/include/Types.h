#pragma once

#include "unitree/dds_wrapper/robots/go2/go2.h"

using LowCmd_t = unitree::robot::go2::publisher::LowCmd;
using LowState_t = unitree::robot::go2::subscription::LowState;

enum FSMMode
{
    Passive = 1,
    FixStand = 2,
    Velocity = 3,
};
