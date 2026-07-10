cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_config VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(CLI11 CONFIG REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import yaml"
  RESULT_VARIABLE MUJOCO_SIM_CONFIG_PYYAML_RESULT
  ERROR_VARIABLE MUJOCO_SIM_CONFIG_PYYAML_ERROR
)
if(NOT MUJOCO_SIM_CONFIG_PYYAML_RESULT EQUAL 0)
  message(FATAL_ERROR
    "mujoco_sim_config requires PyYAML for ${Python3_EXECUTABLE}: "
    "${MUJOCO_SIM_CONFIG_PYYAML_ERROR}"
  )
endif()

add_library(${PROJECT_NAME} src/runtime.cpp)
add_library(mujoco_sim::config ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    CLI11::CLI11
    yaml-cpp::yaml-cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME config)

set(MUJOCO_SIM_CONFIG_GENERATOR
  "${CMAKE_CURRENT_SOURCE_DIR}/generator/mujoco_sim_config_generator.py"
)
set(MUJOCO_SIM_CONFIG_TEMPLATE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/generator/templates"
)
file(GLOB MUJOCO_SIM_CONFIG_GENERATOR_DEPENDENCIES
  "${CMAKE_CURRENT_SOURCE_DIR}/generator/mujoco_sim_config_codegen/*.py"
)
include(cmake/mujoco_sim_generate_config.cmake)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(PROGRAMS generator/mujoco_sim_config_generator.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/generator
)
install(DIRECTORY generator/mujoco_sim_config_codegen
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/generator
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY generator/templates/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/generator/templates
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
  cmake/mujoco_sim_generate_config.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  add_test(
    NAME config_generator_test
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
            -s "${CMAKE_CURRENT_SOURCE_DIR}/test"
            -p "generator_test.py"
  )
  set_tests_properties(config_generator_test PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
  )

  mujoco_sim_generate_config(
    TARGET runtime_test_config
    SCHEMA test/schema/runtime.yaml
  )
  add_executable(config_runtime_test test/runtime_test.cpp)
  target_link_libraries(config_runtime_test PRIVATE runtime_test_config)
  add_test(NAME config_runtime_test COMMAND config_runtime_test)

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(
    NAME config_package_consumer_test
    COMMAND "${CMAKE_COMMAND}"
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )
endif()
