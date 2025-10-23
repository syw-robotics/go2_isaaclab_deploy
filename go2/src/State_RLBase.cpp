#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <cstdio>
#include <chrono>
#include <cmath>
#include <algorithm>

State_RLBase::State_RLBase(int state_mode, std::string state_string, std::string policy_dir, std::string policy_name)
: FSMState(state_mode, state_string) 
{
    spdlog::info("Initializing State_{}...", state_string);
    auto cfg = param::config["FSM"][state_string];
    
    // Load joystick filter rate parameter
    joystick_rate = cfg["joystick_rate"].as<float>();
    spdlog::info("Joystick filter rate: {}", joystick_rate);
    
    // Initialize filtered velocity values
    filtered_lin_vel_x = 0.0f;
    filtered_lin_vel_y = 0.0f;
    filtered_ang_vel_z = 0.0f;
    
    // Use command line parameters if provided, otherwise use config.yaml values
    if (policy_dir.empty()) {
        policy_dir = cfg["policy_dir"].as<std::string>();
    }
    if (policy_name.empty()) {
        policy_name = cfg["policy_name"].as<std::string>();
    }
    std::filesystem::path policy_yaml_path = param::proj_dir / "config" / policy_dir / "policy.yaml";
    std::filesystem::path policy_path = param::proj_dir / "config" / policy_dir / policy_name;

    spdlog::info("Loading policy.yaml from: {}", policy_yaml_path.string());
    spdlog::info("Loading policy onnx from: {}", policy_path.string());
    
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_yaml_path),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_path);

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            (int)FSMMode::Passive
        )
    );
    
    // Add L1 button protection to enter passive mode
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return FSMState::lowstate->joystick.LB.on_pressed; },
            (int)FSMMode::Passive
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
    
    // Control print frequency, update every 1 second
    static auto last_print_time = std::chrono::high_resolution_clock::now();
    static float last_lin_vel_x = 0.0f, last_lin_vel_y = 0.0f, last_ang_vel_z = 0.0f;
    
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_print_time);
    
    if (time_diff.count() >= 200) {  // Update every 200ms
        // Get raw joystick input and apply first-order differential filter
        auto & joystick = env->robot->data.joystick;
        auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
        
        float raw_lin_vel_x = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
        float raw_lin_vel_y = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
        float raw_ang_vel_z = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());
        
        // Apply first-order differential filter: filtered = filtered + rate * (raw - filtered)
        filtered_lin_vel_x += joystick_rate * (raw_lin_vel_x - filtered_lin_vel_x);
        filtered_lin_vel_y += joystick_rate * (raw_lin_vel_y - filtered_lin_vel_y);
        filtered_ang_vel_z += joystick_rate * (raw_ang_vel_z - filtered_ang_vel_z);
        
        // Store filtered values in ArticulationData for policy observations
        env->robot->data.filtered_lin_vel_x = filtered_lin_vel_x;
        env->robot->data.filtered_lin_vel_y = filtered_lin_vel_y;
        env->robot->data.filtered_ang_vel_z = filtered_ang_vel_z;
        
        // Only print if velocity changed significantly (threshold 0.01)
        if (std::abs(filtered_lin_vel_x - last_lin_vel_x) > 0.01f || 
            std::abs(filtered_lin_vel_y - last_lin_vel_y) > 0.01f || 
            std::abs(filtered_ang_vel_z - last_ang_vel_z) > 0.01f) {
            
            printf("\rCommand Velocity - Lin_X: %.3f, Lin_Y: %.3f, Ang_Z: %.3f    ", 
                   filtered_lin_vel_x, filtered_lin_vel_y, filtered_ang_vel_z);
            fflush(stdout);  // Force flush output buffer
            
            last_lin_vel_x = filtered_lin_vel_x;
            last_lin_vel_y = filtered_lin_vel_y;
            last_ang_vel_z = filtered_ang_vel_z;
        }
        
        last_print_time = current_time;
    }
}