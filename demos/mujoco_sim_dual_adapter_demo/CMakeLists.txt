cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_dual_adapter_demo VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

find_program(WBR_ADAPTER_EXECUTABLE
  NAMES mujoco-sim-wbr-lqr-adapter
  REQUIRED
)
find_program(DOUBLE_INVERTED_ADAPTER_EXECUTABLE
  NAMES mujoco-sim-double-inverted-control-demo
  REQUIRED
)

get_filename_component(WBR_BIN_DIR "${WBR_ADAPTER_EXECUTABLE}" DIRECTORY)
get_filename_component(WBR_PREFIX "${WBR_BIN_DIR}" DIRECTORY)
set(WBR_DATA_DIR
  "${WBR_PREFIX}/${CMAKE_INSTALL_DATADIR}/mujoco_sim_wbr_lqr_demo")
set(WBR_MJCF_DIR "${WBR_DATA_DIR}/mjcf")
get_filename_component(DOUBLE_INVERTED_BIN_DIR
  "${DOUBLE_INVERTED_ADAPTER_EXECUTABLE}" DIRECTORY)
get_filename_component(DOUBLE_INVERTED_PREFIX
  "${DOUBLE_INVERTED_BIN_DIR}" DIRECTORY)
set(DOUBLE_INVERTED_DATA_DIR
  "${DOUBLE_INVERTED_PREFIX}/${CMAKE_INSTALL_DATADIR}/mujoco_sim_double_inverted_control_demo")
set(DOUBLE_INVERTED_MJCF_DIR "${DOUBLE_INVERTED_DATA_DIR}/mjcf")

foreach(required_file IN ITEMS
    "${WBR_MJCF_DIR}/mjmodel.xml"
    "${WBR_DATA_DIR}/config/wbr_lqr.yaml"
    "${WBR_DATA_DIR}/UPSTREAM.md"
    "${DOUBLE_INVERTED_MJCF_DIR}/scene.xml"
    "${DOUBLE_INVERTED_DATA_DIR}/config/mujoco_sim_double_inverted_control_demo.yaml")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "required dual-adapter demo dependency asset is missing: ${required_file}")
  endif()
endforeach()

set(DEMO_SHARE_DIR "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}")
install(DIRECTORY config/ DESTINATION "${DEMO_SHARE_DIR}/config")
install(DIRECTORY mjcf/ DESTINATION "${DEMO_SHARE_DIR}/mjcf")
install(DIRECTORY "${WBR_MJCF_DIR}/"
  DESTINATION "${DEMO_SHARE_DIR}/mjcf/wbr")
install(DIRECTORY "${DOUBLE_INVERTED_MJCF_DIR}/"
  DESTINATION "${DEMO_SHARE_DIR}/mjcf/double_inverted")
install(FILES "${WBR_DATA_DIR}/config/wbr_lqr.yaml"
  DESTINATION "${DEMO_SHARE_DIR}/config"
  RENAME wbr_lqr_adapter.yaml)
install(FILES
  "${DOUBLE_INVERTED_DATA_DIR}/config/mujoco_sim_double_inverted_control_demo.yaml"
  DESTINATION "${DEMO_SHARE_DIR}/config"
  RENAME double_inverted_adapter.yaml)
install(FILES "${WBR_DATA_DIR}/UPSTREAM.md"
  DESTINATION "${DEMO_SHARE_DIR}"
  RENAME WBR_UPSTREAM.md)
install(PROGRAMS scripts/launch_dual_adapter_demo.sh
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  RENAME mujoco-sim-dual-adapter-demo-launch)
install(FILES README.md DESTINATION "${DEMO_SHARE_DIR}")

include(CTest)
if(BUILD_TESTING)
  find_program(MUJOCO_SIM_SERVER_EXECUTABLE
    NAMES mujoco-sim-server
    REQUIRED
  )

  set(TEST_MJCF_DIR "${CMAKE_CURRENT_BINARY_DIR}/test_mjcf")
  file(REMOVE_RECURSE "${TEST_MJCF_DIR}")
  file(MAKE_DIRECTORY "${TEST_MJCF_DIR}/wbr" "${TEST_MJCF_DIR}/double_inverted")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/mjcf/scene.xml"
    "${TEST_MJCF_DIR}/scene.xml"
    COPYONLY)
  file(COPY "${WBR_MJCF_DIR}/"
    DESTINATION "${TEST_MJCF_DIR}/wbr")
  file(COPY "${DOUBLE_INVERTED_MJCF_DIR}/"
    DESTINATION "${TEST_MJCF_DIR}/double_inverted")

  add_test(NAME dual_adapter_launcher_lifecycle
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/launcher_lifecycle_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_dual_adapter_demo.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fake_child.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}/config/dual_adapter_server.yaml"
      "${WBR_DATA_DIR}/config/wbr_lqr.yaml"
      "${DOUBLE_INVERTED_DATA_DIR}/config/mujoco_sim_double_inverted_control_demo.yaml"
      "${TEST_MJCF_DIR}/scene.xml")
  set_tests_properties(dual_adapter_launcher_lifecycle PROPERTIES TIMEOUT 30)

  add_test(NAME dual_adapter_headless_e2e
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_dual_adapter.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_dual_adapter_demo.sh"
      "${MUJOCO_SIM_SERVER_EXECUTABLE}"
      "${WBR_ADAPTER_EXECUTABLE}"
      "${DOUBLE_INVERTED_ADAPTER_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/config/dual_adapter_server.yaml"
      "${WBR_DATA_DIR}/config/wbr_lqr.yaml"
      "${DOUBLE_INVERTED_DATA_DIR}/config/mujoco_sim_double_inverted_control_demo.yaml"
      "${TEST_MJCF_DIR}/scene.xml"
      headless 6000)
  set_tests_properties(dual_adapter_headless_e2e PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
    LABELS e2e
    TIMEOUT 90)

  get_filename_component(MUJOCO_SIM_SERVER_BIN_DIR
    "${MUJOCO_SIM_SERVER_EXECUTABLE}" DIRECTORY)
  get_filename_component(MUJOCO_SIM_SERVER_PREFIX
    "${MUJOCO_SIM_SERVER_BIN_DIR}" DIRECTORY)
  set(MUJOCO_SIM_SERVER_TARGETS_FILE
    "${MUJOCO_SIM_SERVER_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_sim_server/mujoco_sim_serverTargets.cmake")
  set(MUJOCO_SIM_SERVER_HAS_GUI OFF)
  if(EXISTS "${MUJOCO_SIM_SERVER_TARGETS_FILE}")
    file(STRINGS "${MUJOCO_SIM_SERVER_TARGETS_FILE}" MUJOCO_SIM_SERVER_GUI_DEFINITION
      REGEX "MUJOCO_SIM_HAS_GUI=.*BOOL:ON")
    if(MUJOCO_SIM_SERVER_GUI_DEFINITION)
      set(MUJOCO_SIM_SERVER_HAS_GUI ON)
    endif()
  endif()
  if(MUJOCO_SIM_SERVER_HAS_GUI)
    find_program(XVFB_RUN_EXECUTABLE NAMES xvfb-run REQUIRED)
    add_test(NAME dual_adapter_gui_e2e
      COMMAND "${XVFB_RUN_EXECUTABLE}" -a
        bash "${CMAKE_CURRENT_SOURCE_DIR}/test/e2e_dual_adapter.sh"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_dual_adapter_demo.sh"
        "${MUJOCO_SIM_SERVER_EXECUTABLE}"
        "${WBR_ADAPTER_EXECUTABLE}"
        "${DOUBLE_INVERTED_ADAPTER_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/config/dual_adapter_server.yaml"
        "${WBR_DATA_DIR}/config/wbr_lqr.yaml"
        "${DOUBLE_INVERTED_DATA_DIR}/config/mujoco_sim_double_inverted_control_demo.yaml"
        "${TEST_MJCF_DIR}/scene.xml"
        gui 200)
    set_tests_properties(dual_adapter_gui_e2e PROPERTIES LABELS e2e TIMEOUT 60)
  endif()
endif()
