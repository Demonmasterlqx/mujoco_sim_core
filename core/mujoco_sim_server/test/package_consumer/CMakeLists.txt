cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_server_package_consumer LANGUAGES CXX)

find_package(mujoco_sim_server CONFIG REQUIRED)

add_executable(package_consumer main.cpp)
target_link_libraries(package_consumer PRIVATE mujoco_sim::server)
