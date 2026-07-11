cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_server VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(eCAL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(mujoco_sim_config CONFIG REQUIRED)
find_package(mujoco_sim_core CONFIG REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(PNG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Threads REQUIRED)

set(MUJOCO_SIMULATE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_core/mujoco/linux/x86/mujoco-3.10.0/simulate"
)

add_library(${PROJECT_NAME}_lib
  src/ecal_server.cpp
  src/lodepng_compat.cpp
  src/server_logging.cpp
  src/server_runtime.cpp
  src/sim_viewer.cpp
  src/server_config.cpp
  "${MUJOCO_SIMULATE_ROOT}/glfw_adapter.cc"
  "${MUJOCO_SIMULATE_ROOT}/glfw_dispatch.cc"
  "${MUJOCO_SIMULATE_ROOT}/platform_ui_adapter.cc"
  "${MUJOCO_SIMULATE_ROOT}/simulate.cc"
)
add_library(mujoco_sim::server ALIAS ${PROJECT_NAME}_lib)
mujoco_sim_generate_config(
  TARGET server_config_generated
  SCHEMA config/server_config.yaml
  OUTPUT_NAME server_config_generated
  OWNER_TARGET ${PROJECT_NAME}_lib
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
  PRIVATE
    "${MUJOCO_SIMULATE_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    mujoco_sim::core
    mujoco_sim::protocol
    eCAL::core
  PRIVATE
    glfw
    mujoco::mujoco
    PNG::PNG
    spdlog::spdlog
    Threads::Threads
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES EXPORT_NAME server)

add_executable(mujoco-sim-server src/main.cpp)
target_link_libraries(mujoco-sim-server PRIVATE ${PROJECT_NAME}_lib)
set_target_properties(mujoco-sim-server PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN/../../mujoco_sim_core/lib"
)

install(TARGETS ${PROJECT_NAME}_lib mujoco-sim-server
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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
endif()

add_subdirectory(tools/status)

if(BUILD_TESTING)
  add_executable(server_test test/server_test.cpp)
  target_include_directories(server_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(server_test PRIVATE
    ${PROJECT_NAME}_lib
    GTest::gtest_main
    spdlog::spdlog
  )
  add_test(NAME server_test COMMAND server_test)

  add_executable(png_writer_test test/png_writer_test.cpp)
  target_include_directories(png_writer_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(png_writer_test PRIVATE
    ${PROJECT_NAME}_lib
    GTest::gtest_main
    PNG::PNG
  )
  add_test(NAME png_writer_test COMMAND png_writer_test)

  add_test(
    NAME server_help
    COMMAND ${CMAKE_COMMAND}
            "-DSERVER_EXECUTABLE=$<TARGET_FILE:mujoco-sim-server>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/test/server_help_test.cmake"
  )

  add_executable(server_logging_probe test/server_logging_probe.cpp)
  target_link_libraries(server_logging_probe PRIVATE
    eCAL::core
    mujoco_sim::protocol
  )
  add_test(
    NAME server_logging_e2e
    COMMAND bash
            "${CMAKE_CURRENT_SOURCE_DIR}/test/server_logging_e2e.sh"
            $<TARGET_FILE:mujoco-sim-server>
            $<TARGET_FILE:server_logging_probe>
            "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  set_tests_properties(server_logging_e2e PROPERTIES TIMEOUT 15)

  add_executable(server_lifecycle_probe test/server_lifecycle_probe.cpp)
  target_link_libraries(server_lifecycle_probe PRIVATE
    eCAL::core
    mujoco_sim::protocol
  )
  add_test(
    NAME server_lifecycle_e2e
    COMMAND bash
            "${CMAKE_CURRENT_SOURCE_DIR}/test/server_lifecycle_e2e.sh"
            $<TARGET_FILE:mujoco-sim-server>
            $<TARGET_FILE:server_lifecycle_probe>
            "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_core/test/testdata/dual_actuator.xml"
  )
  set_tests_properties(server_lifecycle_e2e PROPERTIES TIMEOUT 25)

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(
    NAME server_package_consumer
    COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )

  set(MUJOCO_SIM_GUI_TEST_PREFIX)
  if(NOT DEFINED ENV{DISPLAY} OR "$ENV{DISPLAY}" STREQUAL "")
    find_program(XVFB_RUN_EXECUTABLE NAMES xvfb-run REQUIRED)
    set(MUJOCO_SIM_GUI_TEST_PREFIX "${XVFB_RUN_EXECUTABLE}" -a)
  endif()

  add_executable(sim_viewer_test test/sim_viewer_test.cpp)
  target_include_directories(sim_viewer_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(sim_viewer_test PRIVATE
    ${PROJECT_NAME}_lib
    GTest::gtest_main
  )
  target_compile_definitions(sim_viewer_test PRIVATE
    MUJOCO_SIM_SERVER_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  add_test(
    NAME sim_viewer_test
    COMMAND ${MUJOCO_SIM_GUI_TEST_PREFIX} $<TARGET_FILE:sim_viewer_test>
  )
  set_tests_properties(sim_viewer_test PROPERTIES TIMEOUT 15)

  add_test(
    NAME server_gui_autonomous_run
    COMMAND
      ${MUJOCO_SIM_GUI_TEST_PREFIX}
      bash
      "${CMAKE_CURRENT_SOURCE_DIR}/test/gui_startup_timeout_test.sh"
      $<TARGET_FILE:mujoco-sim-server>
      "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  set_tests_properties(server_gui_autonomous_run PROPERTIES TIMEOUT 15)
endif()
