cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_hardware_interface VERSION 0.1.0 LANGUAGES CXX)

option(MUJOCO_SIM_BUILD_DOCS
  "Build warning-clean hardware-interface documentation with Doxygen" OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_resource.cpp
  src/mujoco_sim_system.cpp
  src/mujoco_sim_system_io.cpp
  src/plugin.cpp
  src/resource.cpp
  src/sensor_resource.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  rosgraph_msgs
  std_msgs
  tf2_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} mujoco_sim::adapter_sdk)

pluginlib_export_plugin_description_file(
  hardware_interface mujoco_sim_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES Readme.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  hardware_interface
  mujoco_sim_adapter_sdk
  pluginlib
  rclcpp
  realtime_tools
  rosgraph_msgs
  std_msgs
  tf2_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(resource_test test/resource_test.cpp)
  target_include_directories(resource_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_link_libraries(resource_test ${PROJECT_NAME})

  ament_add_gtest(resource_publisher_test test/resource_publisher_test.cpp)
  target_include_directories(resource_publisher_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  ament_target_dependencies(resource_publisher_test rclcpp std_msgs tf2_msgs tf2_ros)
  target_link_libraries(resource_publisher_test ${PROJECT_NAME})

  ament_add_gtest(plugin_load_test test/plugin_load_test.cpp)
  ament_target_dependencies(plugin_load_test hardware_interface pluginlib)

  ament_add_gtest(system_interface_test test/system_interface_test.cpp)
  ament_target_dependencies(system_interface_test hardware_interface rclcpp)
  target_link_libraries(system_interface_test ${PROJECT_NAME})

  set(ADAPTER_SDK_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_sim_adapter_sdk")
  ament_add_gtest(system_io_test
    test/system_io_test.cpp
    "${ADAPTER_SDK_SOURCE_DIR}/test/controller_client_test_peer.cpp"
  )
  target_include_directories(system_io_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${ADAPTER_SDK_SOURCE_DIR}/src"
  )
  ament_target_dependencies(system_io_test hardware_interface rclcpp)
  target_link_libraries(system_io_test ${PROJECT_NAME})

  configure_file(
    test/package_consumer_test.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
    @ONLY
  )
  add_test(
    NAME package_consumer_test
    COMMAND "${CMAKE_COMMAND}"
            -P "${CMAKE_CURRENT_BINARY_DIR}/package_consumer_test.cmake"
  )

  # The repository .clang-format is authoritative. ROS uncrustify/cpplint use
  # a conflicting style, and existing first-party files intentionally do not
  # carry per-file copyright banners.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(MUJOCO_SIM_BUILD_DOCS)
  find_package(Doxygen REQUIRED)
  set(DOXYGEN_EXTRACT_ALL NO)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_FILE_PATTERNS "*.hpp")
  set(DOXYGEN_RECURSIVE YES)
  set(DOXYGEN_WARN_IF_UNDOCUMENTED YES)
  set(DOXYGEN_WARN_AS_ERROR FAIL_ON_WARNINGS)
  set(DOXYGEN_QUIET YES)
  doxygen_add_docs(mujoco_sim_hardware_interface_docs
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    COMMENT "Generate warning-clean hardware-interface documentation"
  )
endif()

ament_package()
