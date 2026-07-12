cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_sensor VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(mujoco_sim_core CONFIG REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(OSMesa REQUIRED IMPORTED_TARGET osmesa)

add_library(${PROJECT_NAME}
  src/offscreen_renderer.cpp
  src/ray_lidar.cpp
  src/source_profile.cpp
)
add_library(mujoco_sim::sensor ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mujoco_sim::core
    mujoco_sim::protocol
  PRIVATE
    mujoco::mujoco
    ${CMAKE_DL_LIBS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${EGL_INCLUDE_DIRS}
  ${OSMesa_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME sensor)

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

  add_executable(sensor_test test/sensor_test.cpp)
  target_link_libraries(sensor_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco::mujoco
  )
  target_compile_definitions(sensor_test PRIVATE
    MUJOCO_SIM_SENSOR_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/test/testdata/sensor_scene.xml"
  )
  add_test(
    NAME sensor_test
    COMMAND sensor_test
            "--gtest_filter=-SensorTest.ForcedOsMesaIsHeadlessAndRenders"
  )
  add_test(
    NAME sensor_osmesa_test
    COMMAND sensor_test
            "--gtest_filter=SensorTest.ForcedOsMesaIsHeadlessAndRenders"
  )
  set_tests_properties(sensor_test sensor_osmesa_test PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
  )

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(
    NAME sensor_package_consumer_test
    COMMAND "${CMAKE_COMMAND}"
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )
endif()
