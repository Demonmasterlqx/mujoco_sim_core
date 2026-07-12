cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_protocol VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Protobuf REQUIRED)

set(PROTO_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/proto/mujoco_sim/v1/sync.proto"
)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated/proto")

add_library(${PROJECT_NAME}
  ${PROTO_FILES}
  src/stream_frame.cpp
)
add_library(mujoco_sim::protocol ALIAS ${PROJECT_NAME})

protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  PROTOC_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
  IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/proto"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated/proto>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC protobuf::libprotobuf
)
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME protocol)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated/proto/"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.pb.h"
)
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto)

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
  add_executable(protocol_test test/protocol_test.cpp)
  target_link_libraries(protocol_test PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME protocol_test COMMAND protocol_test)
endif()
