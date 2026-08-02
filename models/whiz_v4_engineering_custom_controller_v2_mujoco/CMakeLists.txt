cmake_minimum_required(VERSION 3.22)

project(whiz_v4_engineering_custom_controller_v2_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES whiz_v4_engineering_custom_controller_v2.urdf DESTINATION share/${PROJECT_NAME})
install(FILES whiz_v4_engineering_custom_controller_v2.xml DESTINATION share/${PROJECT_NAME})

ament_package()
