cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_server VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(CLI11 CONFIG REQUIRED)
find_package(eCAL REQUIRED)
find_package(mujoco_sim_core CONFIG REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/ecal_server.cpp
  src/server_config.cpp
)
add_library(mujoco_sim::server ALIAS ${PROJECT_NAME}_lib)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    mujoco_sim::core
    mujoco_sim::protocol
    eCAL::core
  PRIVATE
    CLI11::CLI11
    yaml-cpp::yaml-cpp
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
  add_executable(server_test test/server_test.cpp)
  target_link_libraries(server_test PRIVATE ${PROJECT_NAME}_lib GTest::gtest_main)
  add_test(NAME server_test COMMAND server_test)

  add_test(
    NAME server_help
    COMMAND ${CMAKE_COMMAND}
            "-DSERVER_EXECUTABLE=$<TARGET_FILE:mujoco-sim-server>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/test/server_help_test.cmake"
  )

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
endif()
