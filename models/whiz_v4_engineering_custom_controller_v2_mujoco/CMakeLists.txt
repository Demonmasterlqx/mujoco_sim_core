cmake_minimum_required(VERSION 3.22)

project(whiz_v4_engineering_custom_controller_v2_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES whiz_v4_engineering_custom_controller_v2.urdf DESTINATION share/${PROJECT_NAME})
install(FILES whiz_v4_engineering_custom_controller_v2.xml DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mujoco_sim_core CONFIG REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  ament_add_gtest(model_contract_test test/model_contract_test.cpp)
  target_compile_features(model_contract_test PRIVATE cxx_std_20)
  target_compile_definitions(model_contract_test PRIVATE
    WHIZ_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/whiz_v4_engineering_custom_controller_v2.xml")
  target_link_libraries(model_contract_test mujoco::mujoco)

  add_test(
    NAME model_config_test
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/model_config_test.py"
            "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(model_config_test PROPERTIES TIMEOUT 20)

  add_test(
    NAME package_install_test
    COMMAND "${CMAKE_COMMAND}"
            -DBUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DINSTALL_STAGE=${CMAKE_CURRENT_BINARY_DIR}/package_install_stage
            -DINSTALLED_PREFIX=${CMAKE_INSTALL_PREFIX}
            -DMODEL_TEST=$<TARGET_FILE:model_contract_test>
            -DXACRO_EXECUTABLE=${XACRO_EXECUTABLE}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/test/package_install_test.cmake"
  )
  set_tests_properties(package_install_test PROPERTIES TIMEOUT 30)
endif()

ament_package()
