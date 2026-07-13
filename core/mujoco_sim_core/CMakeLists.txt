cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(mujoco_sim_protocol CONFIG REQUIRED)

option(MUJOCO_SIM_USE_BUNDLED_MUJOCO
  "Use the MuJoCo distribution bundled in this repository" ON)
option(MUJOCO_SIM_BUILD_PERFORMANCE_BENCHMARK
  "Build the phase-one deterministic performance benchmark" OFF)

if(MUJOCO_SIM_USE_BUNDLED_MUJOCO)
  set(MUJOCO_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/mujoco/linux/x86/mujoco-3.10.0"
    CACHE PATH "Path to the bundled MuJoCo distribution"
  )
  set(MUJOCO_LIBRARY "${MUJOCO_ROOT}/lib/libmujoco.so.3.10.0")
  if(NOT EXISTS "${MUJOCO_LIBRARY}")
    message(FATAL_ERROR "MuJoCo library not found: ${MUJOCO_LIBRARY}")
  endif()
  add_library(mujoco_vendor SHARED IMPORTED GLOBAL)
  add_library(mujoco::mujoco ALIAS mujoco_vendor)
  set_target_properties(mujoco_vendor PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include"
  )
else()
  find_package(mujoco CONFIG REQUIRED)
endif()

add_library(${PROJECT_NAME}
  src/simulation_core.cpp
)
add_library(mujoco_sim::core ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC mujoco_sim::protocol
  PRIVATE mujoco::mujoco
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  EXPORT_NAME core
  INSTALL_RPATH "$ORIGIN"
)
if(MUJOCO_SIM_USE_BUNDLED_MUJOCO)
  set_property(TARGET ${PROJECT_NAME} PROPERTY BUILD_RPATH "${MUJOCO_ROOT}/lib")
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(MUJOCO_SIM_USE_BUNDLED_MUJOCO)
  install(DIRECTORY "${MUJOCO_ROOT}/include/mujoco"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(FILES "${MUJOCO_LIBRARY}" DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

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
  add_executable(core_test test/simulation_core_test.cpp)
  target_link_libraries(core_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco::mujoco
  )
  target_compile_definitions(core_test PRIVATE
    MUJOCO_SIM_CORE_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/test/testdata/simple_actuator.xml"
    MUJOCO_SIM_CORE_DUAL_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/test/testdata/dual_actuator.xml"
  )
  if(MUJOCO_SIM_USE_BUNDLED_MUJOCO)
    set_target_properties(core_test PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}/lib")
  endif()
  add_test(NAME core_test COMMAND core_test)
endif()

if(MUJOCO_SIM_BUILD_PERFORMANCE_BENCHMARK)
  add_executable(core_performance_benchmark test/performance_benchmark.cpp)
  target_link_libraries(core_performance_benchmark PRIVATE
    ${PROJECT_NAME} mujoco::mujoco)
  if(MUJOCO_SIM_USE_BUNDLED_MUJOCO)
    set_target_properties(core_performance_benchmark PROPERTIES
      BUILD_RPATH "${MUJOCO_ROOT}/lib")
  endif()
endif()
