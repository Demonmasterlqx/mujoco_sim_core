cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_wbr_lqr_demo_package_consumer LANGUAGES CXX)

find_package(mujoco_sim_wbr_lqr_demo CONFIG REQUIRED)

if(NOT EXISTS "${mujoco_sim_wbr_lqr_demo_MJCF_DIR}/scene.xml")
  message(FATAL_ERROR "Installed WBR scene.xml was not found")
endif()

add_executable(package_consumer main.cpp)
target_link_libraries(package_consumer PRIVATE mujoco_sim::wbr_lqr_demo)
