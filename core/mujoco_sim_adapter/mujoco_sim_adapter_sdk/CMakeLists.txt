cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_adapter_sdk VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(eCAL REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)

add_library(${PROJECT_NAME}
  src/ecal_runtime.cpp
  src/observer_client.cpp
  src/sensor_stream_client.cpp
  src/sync_adapter_client.cpp
)
add_library(mujoco_sim::adapter_sdk ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC mujoco_sim::protocol
  PUBLIC eCAL::core
)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME adapter_sdk)

install(TARGETS ${PROJECT_NAME}
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
  add_executable(adapter_sdk_test
    test/observer_client_test.cpp
    test/sensor_stream_client_test.cpp
    test/sync_adapter_client_test.cpp
  )
  target_link_libraries(adapter_sdk_test PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME adapter_sdk_test COMMAND adapter_sdk_test)

  add_executable(ecal_runtime_static_lifetime_test
    test/ecal_runtime_static_lifetime_test.cpp
  )
  target_include_directories(ecal_runtime_static_lifetime_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_link_libraries(ecal_runtime_static_lifetime_test PRIVATE
    ${PROJECT_NAME}
  )
  add_test(NAME ecal_runtime_static_lifetime_test
    COMMAND ecal_runtime_static_lifetime_test
  )

  add_executable(sensor_stream_ecal_test
    test/sensor_stream_ecal_test.cpp
  )
  target_link_libraries(sensor_stream_ecal_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME sensor_stream_ecal_test COMMAND sensor_stream_ecal_test)
  set_tests_properties(sensor_stream_ecal_test PROPERTIES TIMEOUT 15)

  find_package(mujoco_sim_server CONFIG REQUIRED)
  if(NOT TARGET mujoco_sim::mujoco-sim-server)
    message(FATAL_ERROR "mujoco_sim_server package did not export mujoco-sim-server")
  endif()
  add_executable(sensor_runtime_ecal_test
    test/sensor_runtime_ecal_test.cpp
  )
  target_include_directories(sensor_runtime_ecal_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_server/src"
  )
  target_compile_definitions(sensor_runtime_ecal_test PRIVATE
    MUJOCO_SIM_SENSOR_RUNTIME_ECAL_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_server/test/testdata/sensor_runtime_scene.xml"
  )
  target_link_libraries(sensor_runtime_ecal_test PRIVATE
    ${PROJECT_NAME}
    mujoco_sim::server
    GTest::gtest_main
  )
  add_test(NAME sensor_runtime_ecal_test COMMAND sensor_runtime_ecal_test)
  set_tests_properties(sensor_runtime_ecal_test PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
    TIMEOUT 20
  )

  add_executable(server_terminal_state_ecal_test
    test/server_terminal_state_ecal_test.cpp
  )
  target_compile_definitions(server_terminal_state_ecal_test PRIVATE
    MUJOCO_SIM_SDK_TERMINAL_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_server/test/testdata/sensor_runtime_scene.xml"
  )
  target_link_libraries(server_terminal_state_ecal_test PRIVATE
    ${PROJECT_NAME}
    mujoco_sim::server
    GTest::gtest_main
  )
  add_test(NAME server_terminal_state_ecal_test
    COMMAND server_terminal_state_ecal_test
  )
  set_tests_properties(server_terminal_state_ecal_test PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
    TIMEOUT 20
  )

  add_executable(adapter_sdk_lifecycle_probe
    test/adapter_sdk_lifecycle_probe.cpp
  )
  target_link_libraries(adapter_sdk_lifecycle_probe PRIVATE ${PROJECT_NAME})
  add_test(
    NAME adapter_sdk_lifecycle_e2e
    COMMAND
      bash
      "${CMAKE_CURRENT_SOURCE_DIR}/test/adapter_sdk_lifecycle_e2e.sh"
      "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
      "$<TARGET_FILE:adapter_sdk_lifecycle_probe>"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  set_tests_properties(adapter_sdk_lifecycle_e2e PROPERTIES TIMEOUT 20)

  add_executable(sensor_observer_e2e_probe
    test/sensor_observer_e2e_probe.cpp
  )
  target_link_libraries(sensor_observer_e2e_probe PRIVATE ${PROJECT_NAME})
  add_test(
    NAME sensor_observer_e2e
    COMMAND
      bash
      "${CMAKE_CURRENT_SOURCE_DIR}/test/sensor_observer_e2e.sh"
      "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
      "$<TARGET_FILE:sensor_observer_e2e_probe>"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_server/test/testdata/sensor_runtime_scene.xml"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/sensor_observer_e2e_server.yaml"
  )
  set_tests_properties(sensor_observer_e2e PROPERTIES TIMEOUT 50)
endif()
