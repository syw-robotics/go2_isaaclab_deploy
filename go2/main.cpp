#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n\n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto & joy = FSMState::lowstate->joystick;
    auto fsm = std::make_unique<CtrlFSM>(new State_Passive(FSMMode::Passive));
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return joy.LT.pressed && joy.A.on_pressed; }, // L2 + A
            (int)FSMMode::FixStand
        )
    );
    fsm->add(new State_FixStand(FSMMode::FixStand));
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return joy.start.on_pressed; }, // Start
            FSMMode::Velocity
        )
    );
    // Get policy parameters from command line or use config defaults
    std::string policy_dir = "";
    std::string policy_name = "";
    if (vm.count("policy_dir")) {
        policy_dir = vm["policy_dir"].as<std::string>();
        spdlog::info("Using policy_dir from command line: {}", policy_dir);
    }
    if (vm.count("policy_name")) {
        policy_name = vm["policy_name"].as<std::string>();
        spdlog::info("Using policy_name from command line: {}", policy_name);
    }
    
    fsm->add(new State_RLBase(FSMMode::Velocity, "Velocity", policy_dir, policy_name));

    std::cout << "\n=====================================\n";
    std::cout << "Press [L2 + A] to enter FixStand mode.\n";
    std::cout << "Then press [Start] to start RL controller.\n";
    std::cout << "=====================================\n";
    std::cout << "Press [L1] to enter Passive mode.\n";
    std::cout << "=====================================\n\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

