cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_config_package_consumer LANGUAGES CXX)

find_package(mujoco_sim_config CONFIG REQUIRED)

mujoco_sim_generate_config(
  TARGET consumer_generated
  SCHEMA consumer.yaml
  OUTPUT_NAME consumer_config
)

add_executable(package_consumer main.cpp)
target_link_libraries(package_consumer PRIVATE consumer_generated)
