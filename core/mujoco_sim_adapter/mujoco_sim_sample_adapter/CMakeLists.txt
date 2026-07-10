cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_sample_adapter VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/sample_adapter.cpp
)
add_library(mujoco_sim::sample_adapter ALIAS ${PROJECT_NAME}_lib)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC mujoco_sim::adapter_sdk)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES EXPORT_NAME sample_adapter)

add_executable(mujoco-sim-sample-adapter src/main.cpp)
target_link_libraries(mujoco-sim-sample-adapter PRIVATE ${PROJECT_NAME}_lib)
set_target_properties(mujoco-sim-sample-adapter PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS ${PROJECT_NAME}_lib mujoco-sim-sample-adapter
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
  add_executable(sample_adapter_test test/sample_adapter_test.cpp)
  target_link_libraries(sample_adapter_test PRIVATE ${PROJECT_NAME}_lib GTest::gtest_main)
  add_test(NAME sample_adapter_test COMMAND sample_adapter_test)

  find_package(mujoco_sim_server CONFIG REQUIRED)
  if(NOT TARGET mujoco_sim::mujoco-sim-server)
    message(FATAL_ERROR "mujoco_sim_server package did not export mujoco-sim-server")
  endif()
  add_test(
    NAME sample_adapter_e2e
    COMMAND
      bash
      "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_lockstep.sh"
      "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
      "$<TARGET_FILE:mujoco-sim-sample-adapter>"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  set_tests_properties(sample_adapter_e2e PROPERTIES TIMEOUT 20)
endif()
