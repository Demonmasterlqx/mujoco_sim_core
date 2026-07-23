add_library(mujoco_sim_status_lib STATIC
  status_monitor.cpp
  status_monitor_support.cpp
  status_tool.cpp
)
mujoco_sim_generate_config(
  TARGET status_config_generated
  SCHEMA config/status_config.yaml
  OUTPUT_NAME status_config_generated
  OWNER_TARGET mujoco_sim_status_lib
)
target_compile_features(mujoco_sim_status_lib PUBLIC cxx_std_20)
target_include_directories(mujoco_sim_status_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mujoco_sim_status_lib
  PUBLIC mujoco_sim::protocol
  PRIVATE eCAL::core Threads::Threads
)

add_executable(mujoco-sim-status main.cpp)
target_link_libraries(mujoco-sim-status PRIVATE mujoco_sim_status_lib)
install(TARGETS mujoco-sim-status RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_executable(status_tool_test test/status_tool_test.cpp)
  target_link_libraries(status_tool_test PRIVATE
    mujoco_sim_status_lib GTest::gtest_main
  )
  add_test(NAME status_tool_test COMMAND status_tool_test)

  add_test(
    NAME status_tool_e2e
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/status_tool_e2e.sh"
            $<TARGET_FILE:mujoco-sim-server>
            $<TARGET_FILE:mujoco-sim-status>
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco_sim_core/test/testdata/simple_actuator.xml"
  )
  set_tests_properties(status_tool_e2e PROPERTIES TIMEOUT 40)
endif()
