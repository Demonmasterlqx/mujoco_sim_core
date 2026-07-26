cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_double_inverted_control_demo VERSION 0.2.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
find_package(mujoco_sim_config CONFIG REQUIRED)

mujoco_sim_generate_config(
    TARGET mujoco_sim_double_inverted_control_config
    SCHEMA src/mujoco_sim_double_inverted_control_demo.yaml
    OUTPUT_NAME mujoco_sim_double_inverted_control_config
)

add_library(${PROJECT_NAME}
  src/mujoco_sim_double_inverted_control_demo.cpp
  src/control.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    mujoco_sim_double_inverted_control_config
    mujoco_sim::adapter_sdk
)

add_executable(mujoco-sim-double-inverted-control-demo src/main.cpp)
target_link_libraries(mujoco-sim-double-inverted-control-demo PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} mujoco-sim-double-inverted-control-demo
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY mjcf/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/mjcf)
install(DIRECTORY config/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(mujoco_sim_core CONFIG REQUIRED)

  add_executable(double_inverted_model_contract_test
    test/model_contract_test.cpp
  )
  target_compile_features(double_inverted_model_contract_test PRIVATE cxx_std_20)
  target_compile_definitions(double_inverted_model_contract_test PRIVATE
    DOUBLE_INVERTED_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  target_link_libraries(double_inverted_model_contract_test PRIVATE
    GTest::gtest_main
    mujoco::mujoco
  )
  add_test(NAME double_inverted_model_contract_test
    COMMAND double_inverted_model_contract_test
  )
  set_tests_properties(double_inverted_model_contract_test PROPERTIES TIMEOUT 20)
endif()
