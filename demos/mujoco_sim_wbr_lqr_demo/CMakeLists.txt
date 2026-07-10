cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_wbr_lqr_demo VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
find_package(mujoco_sim_config CONFIG REQUIRED)

add_library(${PROJECT_NAME}
  src/config.cpp
  src/controller_pipeline.cpp
  src/input_script.cpp
  src/interface_mapping.cpp
  src/tick_controller.cpp
)
add_library(mujoco_sim::wbr_lqr_demo ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/wbr_mujoco/controller/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/vendor>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC mujoco_sim::adapter_sdk)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME wbr_lqr_demo)

mujoco_sim_generate_config(
  TARGET wbr_lqr_generated_config
  SCHEMA config/wbr_lqr.schema.yaml
  OUTPUT_NAME wbr_lqr_generated_config
)
add_library(wbr_lqr_app_config STATIC src/app_config.cpp)
target_compile_features(wbr_lqr_app_config PUBLIC cxx_std_20)
target_include_directories(wbr_lqr_app_config PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(wbr_lqr_app_config PUBLIC
  ${PROJECT_NAME}
  wbr_lqr_generated_config
)

add_executable(mujoco-sim-wbr-lqr-adapter src/main.cpp)
target_link_libraries(mujoco-sim-wbr-lqr-adapter PRIVATE wbr_lqr_app_config)
set_target_properties(mujoco-sim-wbr-lqr-adapter PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS ${PROJECT_NAME} mujoco-sim-wbr-lqr-adapter
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  DIRECTORY third_party/wbr_mujoco/controller/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/vendor
)
install(DIRECTORY mjcf/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/mjcf)
install(DIRECTORY config/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)
install(FILES README.md UPSTREAM.md MIGRATION_REPORT.md
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

  add_executable(wbr_pipeline_test
    test/controller_pipeline_test.cpp
    test/pipeline_golden_test.cpp
  )
  target_link_libraries(wbr_pipeline_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME wbr_pipeline_test COMMAND wbr_pipeline_test)

  add_executable(wbr_mapping_test test/interface_mapping_test.cpp)
  target_link_libraries(wbr_mapping_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME wbr_mapping_test COMMAND wbr_mapping_test)

  add_executable(wbr_input_script_test test/input_script_test.cpp)
  target_link_libraries(wbr_input_script_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
  )
  add_test(NAME wbr_input_script_test COMMAND wbr_input_script_test)

  add_executable(wbr_app_config_test test/app_config_test.cpp)
  target_link_libraries(wbr_app_config_test PRIVATE
    wbr_lqr_app_config
    GTest::gtest_main
  )
  target_compile_definitions(wbr_app_config_test PRIVATE
    WBR_LQR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )
  add_test(NAME wbr_app_config_test COMMAND wbr_app_config_test)

  find_package(mujoco_sim_core CONFIG REQUIRED)
  find_package(mujoco_sim_server CONFIG REQUIRED)
  add_executable(wbr_model_contract_test test/model_contract_test.cpp)
  target_link_libraries(wbr_model_contract_test PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco_sim::core
    mujoco::mujoco
  )
  target_compile_definitions(wbr_model_contract_test PRIVATE
    WBR_LQR_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  add_test(NAME wbr_model_contract_test COMMAND wbr_model_contract_test)

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(
    NAME wbr_package_consumer
    COMMAND "${CMAKE_COMMAND}"
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )
  set_tests_properties(wbr_package_consumer PROPERTIES TIMEOUT 60)

  add_test(
    NAME wbr_lqr_e2e
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_wbr_lqr.sh"
            "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
            "$<TARGET_FILE:mujoco-sim-wbr-lqr-adapter>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/wbr_server.yaml"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/wbr_lqr.yaml"
            headless 6000
  )
  set_tests_properties(wbr_lqr_e2e PROPERTIES TIMEOUT 60)

  set(MUJOCO_SIM_WBR_GUI_TEST_PREFIX)
  if(NOT DEFINED ENV{DISPLAY} OR "$ENV{DISPLAY}" STREQUAL "")
    find_program(XVFB_RUN_EXECUTABLE NAMES xvfb-run REQUIRED)
    set(MUJOCO_SIM_WBR_GUI_TEST_PREFIX "${XVFB_RUN_EXECUTABLE}" -a)
  endif()

  add_executable(wbr_gui_render_test test/gui_render_test.cpp)
  target_include_directories(wbr_gui_render_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../core/mujoco_sim_server/src"
  )
  target_link_libraries(wbr_gui_render_test PRIVATE
    mujoco_sim::server
    GTest::gtest_main
    PNG::PNG
  )
  target_compile_definitions(wbr_gui_render_test PRIVATE
    WBR_LQR_MJCF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mjcf"
  )
  add_test(
    NAME wbr_gui_render_test
    COMMAND ${MUJOCO_SIM_WBR_GUI_TEST_PREFIX}
            $<TARGET_FILE:wbr_gui_render_test>
  )
  set_tests_properties(wbr_gui_render_test PROPERTIES TIMEOUT 20)

  add_test(
    NAME wbr_lqr_gui_e2e
    COMMAND ${MUJOCO_SIM_WBR_GUI_TEST_PREFIX}
            bash "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_wbr_lqr.sh"
            "$<TARGET_FILE:mujoco_sim::mujoco-sim-server>"
            "$<TARGET_FILE:mujoco-sim-wbr-lqr-adapter>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/wbr_server.yaml"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/wbr_lqr.yaml"
            gui 200
  )
  set_tests_properties(wbr_lqr_gui_e2e PROPERTIES TIMEOUT 60)
endif()
