cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_hardware_interface_package_consumer LANGUAGES CXX)

find_package(mujoco_sim_hardware_interface CONFIG REQUIRED)

add_executable(package_consumer main.cpp)
target_compile_features(package_consumer PRIVATE cxx_std_20)
target_link_libraries(package_consumer
  PRIVATE
    mujoco_sim_hardware_interface::mujoco_sim_hardware_interface
)
