cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_hardware_interface_demo VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch mjcf urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/command_generator.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(model_contract_test test/model_contract_test.cpp)
  ament_target_dependencies(model_contract_test hardware_interface)
  target_compile_features(model_contract_test PRIVATE cxx_std_20)
  target_compile_definitions(model_contract_test PRIVATE
    DEMO_URDF="${CMAKE_CURRENT_SOURCE_DIR}/urdf/diff_drive.urdf")

  add_test(
    NAME demo_e2e_test
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/demo_e2e_test.py"
  )
  set_tests_properties(demo_e2e_test PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY=;ROS_DOMAIN_ID=173"
    TIMEOUT 35
  )
endif()

ament_package()
