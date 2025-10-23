// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state) : FSMState(state, "FixStand") 
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
        
        // Add L1 button protection to enter passive mode
        this->registered_checks.emplace_back(
            std::make_pair(
                [&]()->bool{ return FSMState::lowstate->joystick.LB.on_pressed; },
                (int)FSMMode::Passive
            )
        );
    }

    void enter()
    {
        // set gain
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }


        // set initial position
        std::vector<float> q0;
        for(int i(0); i < kp.size(); ++i) {
            q0.push_back(lowcmd->msg_.motor_cmd()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        auto q = linear_interpolate(t, ts_, qs_);
        
        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }
    }

private:
    double t0_;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

