if(NOT DEFINED BUILD_DIR OR NOT DEFINED INSTALL_STAGE OR NOT DEFINED INSTALLED_PREFIX OR
   NOT DEFINED MODEL_TEST OR NOT DEFINED XACRO_EXECUTABLE)
  message(FATAL_ERROR "package install test is missing a required path")
endif()

file(REMOVE_RECURSE "${INSTALL_STAGE}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --prefix "${INSTALL_STAGE}"
  RESULT_VARIABLE install_result
  OUTPUT_VARIABLE install_output
  ERROR_VARIABLE install_error
)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "failed to stage package install: ${install_output}${install_error}")
endif()

set(package_relative "share/whiz_v4_engineering_custom_controller_v2_mujoco")
set(share_dir "${INSTALL_STAGE}/${package_relative}")
if(NOT EXISTS "${share_dir}")
  # ament_cmake_symlink_install intentionally keeps its configured prefix.
  set(share_dir "${INSTALLED_PREFIX}/${package_relative}")
endif()
set(model_path "${share_dir}/whiz_v4_engineering_custom_controller_v2.xml")
foreach(relative_path IN ITEMS
    README.md
    whiz_v4_engineering_custom_controller_v2.xml
    whiz_v4_engineering_custom_controller_v2.urdf
    config/controllers.yaml
    config/whiz_v4_engineering_custom_controller_v2.urdf.xacro
    launch/launch_hardware_interface.py
    meshes/base_link.STL
    meshes/j6_link.STL)
  if(NOT EXISTS "${share_dir}/${relative_path}")
    message(FATAL_ERROR "installed package is missing ${relative_path}")
  endif()
endforeach()

execute_process(
  COMMAND "${XACRO_EXECUTABLE}"
          "${share_dir}/config/whiz_v4_engineering_custom_controller_v2.urdf.xacro"
  RESULT_VARIABLE xacro_result
  OUTPUT_QUIET
  ERROR_VARIABLE xacro_error
)
if(NOT xacro_result EQUAL 0)
  message(FATAL_ERROR "installed xacro failed: ${xacro_error}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "WHIZ_MODEL_PATH_OVERRIDE=${model_path}"
          "${MODEL_TEST}"
  RESULT_VARIABLE model_result
  OUTPUT_VARIABLE model_output
  ERROR_VARIABLE model_error
)
if(NOT model_result EQUAL 0)
  message(FATAL_ERROR "installed model contract failed: ${model_output}${model_error}")
endif()
