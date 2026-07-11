cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_adapter_sdk VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(eCAL REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)

add_library(${PROJECT_NAME}
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
  add_executable(adapter_sdk_test test/sync_adapter_client_test.cpp)
  target_link_libraries(adapter_sdk_test PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME adapter_sdk_test COMMAND adapter_sdk_test)

  find_package(mujoco_sim_server CONFIG REQUIRED)
  if(NOT TARGET mujoco_sim::mujoco-sim-server)
    message(FATAL_ERROR "mujoco_sim_server package did not export mujoco-sim-server")
  endif()
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
endif()
