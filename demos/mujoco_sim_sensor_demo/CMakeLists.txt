cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_sensor_demo VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(Eigen3 CONFIG REQUIRED)
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
find_package(mujoco_sim_config CONFIG REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL REQUIRED IMPORTED_TARGET
  pcl_common
  pcl_filters
  pcl_registration
)
find_package(PNG REQUIRED)

add_library(${PROJECT_NAME}
  src/icp_mapper.cpp
  src/imu_synchronizer.cpp
  src/interface_data.cpp
  src/map_output.cpp
  src/stream_processing.cpp
  src/waypoint_controller.cpp
)
add_library(mujoco_sim::sensor_demo ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    mujoco_sim::adapter_sdk
    mujoco_sim::protocol
    PkgConfig::PCL
    PNG::PNG
)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME sensor_demo)

mujoco_sim_generate_config(
  TARGET sensor_demo_generated_config
  SCHEMA config/sensor_demo.schema.yaml
  OUTPUT_NAME sensor_demo_generated_config
)
add_library(sensor_demo_app STATIC
  src/app_config.cpp
  src/process_signal.cpp
)
target_compile_features(sensor_demo_app PUBLIC cxx_std_20)
target_include_directories(sensor_demo_app PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(sensor_demo_app PUBLIC
  ${PROJECT_NAME}
  sensor_demo_generated_config
)

set(sensor_demo_executables
  mujoco-sim-sensor-demo-controller
  mujoco-sim-sensor-demo-state-observer
  mujoco-sim-sensor-demo-camera-consumer
  mujoco-sim-sensor-demo-mapper
)
add_executable(mujoco-sim-sensor-demo-controller src/controller_main.cpp)
add_executable(mujoco-sim-sensor-demo-state-observer
  src/state_observer_main.cpp
)
add_executable(mujoco-sim-sensor-demo-camera-consumer
  src/camera_consumer_main.cpp
)
add_executable(mujoco-sim-sensor-demo-mapper src/mapper_main.cpp)
foreach(executable IN LISTS sensor_demo_executables)
  target_link_libraries(${executable} PRIVATE sensor_demo_app)
  set_target_properties(${executable} PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
  )
endforeach()

install(TARGETS ${PROJECT_NAME} ${sensor_demo_executables}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY mjcf/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/mjcf
)
install(PROGRAMS scripts/launch_sensor_demo.sh
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  RENAME mujoco-sim-sensor-demo-launch
)
install(FILES README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE mujoco_sim::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(mujoco_sim_core CONFIG REQUIRED)
  find_package(mujoco_sim_sensor CONFIG REQUIRED)
  find_package(mujoco_sim_server CONFIG REQUIRED)

  add_executable(sensor_demo_algorithm_test
    test/icp_mapper_test.cpp
    test/map_output_test.cpp
    test/waypoint_controller_test.cpp
  )
  target_link_libraries(sensor_demo_algorithm_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME sensor_demo_algorithm_test COMMAND sensor_demo_algorithm_test)

  add_executable(sensor_demo_app_config_test test/app_config_test.cpp)
  target_include_directories(sensor_demo_app_config_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_compile_definitions(sensor_demo_app_config_test PRIVATE
    SENSOR_DEMO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )
  target_link_libraries(sensor_demo_app_config_test PRIVATE
    sensor_demo_app
    GTest::gtest_main
  )
  add_test(NAME sensor_demo_app_config_test
    COMMAND sensor_demo_app_config_test
  )

  add_executable(sensor_demo_data_pipeline_test test/data_pipeline_test.cpp)
  target_link_libraries(sensor_demo_data_pipeline_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME sensor_demo_data_pipeline_test
    COMMAND sensor_demo_data_pipeline_test
  )

  add_executable(sensor_demo_model_contract_test test/model_contract_test.cpp)
  target_compile_definitions(sensor_demo_model_contract_test PRIVATE
    SENSOR_DEMO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    SENSOR_DEMO_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  target_link_libraries(sensor_demo_model_contract_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco_sim::core
    mujoco_sim::server
    mujoco::mujoco
  )
  add_test(NAME sensor_demo_model_contract_test
    COMMAND sensor_demo_model_contract_test
  )

  add_executable(sensor_demo_render_screenshot_test
    test/render_screenshot_test.cpp
  )
  target_compile_definitions(sensor_demo_render_screenshot_test PRIVATE
    SENSOR_DEMO_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  target_link_libraries(sensor_demo_render_screenshot_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco_sim::core
    mujoco_sim::sensor
  )
  add_test(NAME sensor_demo_render_screenshot_test
    COMMAND sensor_demo_render_screenshot_test
  )
  set_tests_properties(sensor_demo_render_screenshot_test PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
    TIMEOUT 20
  )

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(NAME sensor_demo_package_consumer
    COMMAND "${CMAKE_COMMAND}"
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )
  set_tests_properties(sensor_demo_package_consumer PROPERTIES TIMEOUT 60)

  add_executable(sensor_demo_e2e_quality_score test/e2e_quality_score.cpp)
  target_compile_features(sensor_demo_e2e_quality_score PRIVATE cxx_std_20)
  target_link_libraries(sensor_demo_e2e_quality_score PRIVATE PNG::PNG)
  add_executable(sensor_demo_e2e_quality_score_test
    test/e2e_quality_score_test.cpp
  )
  target_compile_features(sensor_demo_e2e_quality_score_test PRIVATE cxx_std_20)
  target_link_libraries(sensor_demo_e2e_quality_score_test PRIVATE
    GTest::gtest_main
    PNG::PNG
  )
  add_test(NAME sensor_demo_e2e_quality_score_test
    COMMAND sensor_demo_e2e_quality_score_test
  )

  add_test(NAME sensor_demo_launcher_lifecycle_test
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/launcher_lifecycle_test.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_sensor_demo.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/fake_sensor_demo_child.sh"
  )
  set_tests_properties(sensor_demo_launcher_lifecycle_test PROPERTIES
    TIMEOUT 30
  )

  add_test(NAME sensor_demo_gui_stream_lifecycle_test
    COMMAND bash
            "${CMAKE_CURRENT_SOURCE_DIR}/test/gui_stream_lifecycle_test.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/gui_stream_e2e.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/fake_sensor_demo_child.sh"
  )
  set_tests_properties(sensor_demo_gui_stream_lifecycle_test PROPERTIES
    TIMEOUT 30
  )

  add_executable(sensor_demo_e2e_truth_recorder
    test/e2e_truth_recorder.cpp
  )
  target_compile_features(sensor_demo_e2e_truth_recorder PRIVATE cxx_std_20)
  target_link_libraries(sensor_demo_e2e_truth_recorder PRIVATE
    mujoco_sim::adapter_sdk
  )
  add_test(NAME sensor_demo_headless_e2e
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_sensor_demo.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_sensor_demo.sh"
            "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
            "$<TARGET_FILE:mujoco-sim-sensor-demo-controller>"
            "$<TARGET_FILE:mujoco-sim-sensor-demo-state-observer>"
            "$<TARGET_FILE:mujoco-sim-sensor-demo-camera-consumer>"
            "$<TARGET_FILE:mujoco-sim-sensor-demo-mapper>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/sensor_demo_server.yaml"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/sensor_demo.yaml"
            "$<TARGET_FILE:sensor_demo_e2e_quality_score>"
            "$<TARGET_FILE:sensor_demo_e2e_truth_recorder>"
  )
  set_tests_properties(sensor_demo_headless_e2e PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
    LABELS e2e
    TIMEOUT 200
  )

  find_program(XVFB_RUN_EXECUTABLE NAMES xvfb-run REQUIRED)
  set(SENSOR_DEMO_GUI_TEST_PREFIX "${XVFB_RUN_EXECUTABLE}" -a)

  add_executable(sensor_demo_gui_render_test test/gui_render_test.cpp)
  target_compile_definitions(sensor_demo_gui_render_test PRIVATE
    SENSOR_DEMO_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  target_link_libraries(sensor_demo_gui_render_test PRIVATE
    GTest::gtest_main
    mujoco_sim::server
    PNG::PNG
  )
  add_test(NAME sensor_demo_gui_render_test
    COMMAND ${SENSOR_DEMO_GUI_TEST_PREFIX}
            $<TARGET_FILE:sensor_demo_gui_render_test>
  )
  set_tests_properties(sensor_demo_gui_render_test PROPERTIES TIMEOUT 20)

  add_test(NAME sensor_demo_gui_stream_e2e
    COMMAND ${SENSOR_DEMO_GUI_TEST_PREFIX}
            bash "${CMAKE_CURRENT_SOURCE_DIR}/test/gui_stream_e2e.sh"
            "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
            "$<TARGET_FILE:mujoco-sim-sensor-demo-camera-consumer>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/sensor_demo_server.yaml"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/sensor_demo.yaml"
  )
  set_tests_properties(sensor_demo_gui_stream_e2e PROPERTIES
    LABELS e2e
    TIMEOUT 60
  )
endif()
