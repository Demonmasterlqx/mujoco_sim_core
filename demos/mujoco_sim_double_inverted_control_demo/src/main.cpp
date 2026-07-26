#include "mujoco_sim_double_inverted_control_demo/mujoco_sim_double_inverted_control_demo.hpp"

#include <iostream>

int main(int argc, char** argv) {

    std::vector<const char*> arguments;
    arguments.reserve(static_cast<std::size_t>(argc));
    for (int i = 0; i < argc; ++i)
        arguments.push_back(argv[i]);

    std::unique_ptr<mujoco_sim_double_inverted_control_demo::ServerConfig> server_config 
        = std::make_unique<mujoco_sim_double_inverted_control_demo::ServerConfig>();
    std::string error;
    if (!mujoco_sim_double_inverted_control_demo::Parse(argc, arguments.data(), server_config.get(),
                                                    &error)) {
        if (error == mujoco_sim_double_inverted_control_demo::Usage()) {
        std::cout << error;
        return 0;
        }
        std::cerr << error << '\n';
        return 1;
    }
    

    mujoco_sim_double_inverted_control_demo::DoubleInvertedController controller(std::move(server_config));
    if (!controller.RegisterController()) {
        return -1;
    }

    if (!controller.RunControlLoop()) {
        std::cerr << "Control loop failed" << std::endl;
        return -1;
    }

    return 0;
}