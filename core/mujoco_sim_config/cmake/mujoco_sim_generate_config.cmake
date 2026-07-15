include_guard(GLOBAL)

function(mujoco_sim_generate_config)
  set(options)
  set(one_value_args TARGET SCHEMA OUTPUT_NAME OUTPUT_DIRECTORY OWNER_TARGET)
  set(multi_value_args)
  cmake_parse_arguments(
    MUJOCO_CONFIG
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  if(MUJOCO_CONFIG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "mujoco_sim_generate_config received unknown arguments: "
      "${MUJOCO_CONFIG_UNPARSED_ARGUMENTS}"
    )
  endif()
  if(NOT MUJOCO_CONFIG_TARGET)
    message(FATAL_ERROR "mujoco_sim_generate_config requires TARGET")
  endif()
  if(NOT MUJOCO_CONFIG_SCHEMA)
    message(FATAL_ERROR "mujoco_sim_generate_config requires SCHEMA")
  endif()
  if(TARGET "${MUJOCO_CONFIG_TARGET}" AND NOT MUJOCO_CONFIG_OWNER_TARGET)
    message(FATAL_ERROR "target already exists: ${MUJOCO_CONFIG_TARGET}")
  endif()
  if(MUJOCO_CONFIG_OWNER_TARGET AND
     NOT TARGET "${MUJOCO_CONFIG_OWNER_TARGET}")
    message(FATAL_ERROR
      "OWNER_TARGET does not exist: ${MUJOCO_CONFIG_OWNER_TARGET}"
    )
  endif()

  if(NOT IS_ABSOLUTE "${MUJOCO_CONFIG_SCHEMA}")
    get_filename_component(schema
      "${MUJOCO_CONFIG_SCHEMA}"
      ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
    )
  else()
    set(schema "${MUJOCO_CONFIG_SCHEMA}")
  endif()
  if(NOT EXISTS "${schema}")
    message(FATAL_ERROR "configuration schema does not exist: ${schema}")
  endif()

  if(MUJOCO_CONFIG_OUTPUT_NAME)
    set(output_name "${MUJOCO_CONFIG_OUTPUT_NAME}")
  else()
    set(output_name "${MUJOCO_CONFIG_TARGET}")
  endif()
  if(NOT output_name MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
    message(FATAL_ERROR
      "OUTPUT_NAME must be a C/C++ identifier: ${output_name}"
    )
  endif()

  if(MUJOCO_CONFIG_OUTPUT_DIRECTORY)
    get_filename_component(output_directory
      "${MUJOCO_CONFIG_OUTPUT_DIRECTORY}"
      ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}"
    )
  else()
    set(output_directory
      "${CMAKE_CURRENT_BINARY_DIR}/mujoco_sim_config/${MUJOCO_CONFIG_TARGET}"
    )
  endif()
  set(generated_header "${output_directory}/${output_name}.hpp")
  set(generated_source "${output_directory}/${output_name}.cpp")

  if(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
  endif()
  if(NOT MUJOCO_SIM_CONFIG_GENERATOR OR
     NOT EXISTS "${MUJOCO_SIM_CONFIG_GENERATOR}")
    message(FATAL_ERROR
      "MUJOCO_SIM_CONFIG_GENERATOR does not identify an installed generator"
    )
  endif()
  if(NOT MUJOCO_SIM_CONFIG_TEMPLATE_DIR OR
     NOT EXISTS "${MUJOCO_SIM_CONFIG_TEMPLATE_DIR}/config.hpp.in" OR
     NOT EXISTS "${MUJOCO_SIM_CONFIG_TEMPLATE_DIR}/config.cpp.in")
    message(FATAL_ERROR
      "MUJOCO_SIM_CONFIG_TEMPLATE_DIR does not identify installed templates"
    )
  endif()

  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import yaml"
    RESULT_VARIABLE pyyaml_result
    ERROR_VARIABLE pyyaml_error
  )
  if(NOT pyyaml_result EQUAL 0)
    message(FATAL_ERROR
      "mujoco_sim_generate_config requires PyYAML for ${Python3_EXECUTABLE}: "
      "${pyyaml_error}"
    )
  endif()

  add_custom_command(
    OUTPUT "${generated_header}" "${generated_source}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${output_directory}"
    COMMAND "${CMAKE_COMMAND}" -E env "PYTHONDONTWRITEBYTECODE=1"
            "${Python3_EXECUTABLE}" "${MUJOCO_SIM_CONFIG_GENERATOR}"
            --schema "${schema}"
            --header "${generated_header}"
            --source "${generated_source}"
            --header-include "${output_name}.hpp"
            --template-dir "${MUJOCO_SIM_CONFIG_TEMPLATE_DIR}"
    DEPENDS
      "${schema}"
      "${MUJOCO_SIM_CONFIG_GENERATOR}"
      ${MUJOCO_SIM_CONFIG_GENERATOR_DEPENDENCIES}
      "${MUJOCO_SIM_CONFIG_TEMPLATE_DIR}/config.hpp.in"
      "${MUJOCO_SIM_CONFIG_TEMPLATE_DIR}/config.cpp.in"
    COMMENT "Generating ${output_name} from ${schema}"
    VERBATIM
  )
  set_source_files_properties(
    "${generated_header}" "${generated_source}"
    PROPERTIES GENERATED TRUE
  )
  set_source_files_properties("${generated_source}" PROPERTIES
    COMPILE_OPTIONS "-w"
  )

  if(MUJOCO_CONFIG_OWNER_TARGET)
    target_sources("${MUJOCO_CONFIG_OWNER_TARGET}" PRIVATE
      "${generated_header}"
      "${generated_source}"
    )
    target_include_directories("${MUJOCO_CONFIG_OWNER_TARGET}" PRIVATE
      "${output_directory}"
    )
    target_link_libraries("${MUJOCO_CONFIG_OWNER_TARGET}" PRIVATE
      mujoco_sim::config
    )
    set_target_properties("${MUJOCO_CONFIG_OWNER_TARGET}" PROPERTIES
      "MUJOCO_SIM_CONFIG_${MUJOCO_CONFIG_TARGET}_HEADER" "${generated_header}"
      "MUJOCO_SIM_CONFIG_${MUJOCO_CONFIG_TARGET}_SOURCE" "${generated_source}"
    )
  else()
    add_library("${MUJOCO_CONFIG_TARGET}"
      "${generated_header}"
      "${generated_source}"
    )
    target_compile_features("${MUJOCO_CONFIG_TARGET}" PUBLIC cxx_std_20)
    target_include_directories("${MUJOCO_CONFIG_TARGET}"
      PUBLIC "$<BUILD_INTERFACE:${output_directory}>"
    )
    target_link_libraries("${MUJOCO_CONFIG_TARGET}"
      PUBLIC mujoco_sim::config
    )
    set_target_properties("${MUJOCO_CONFIG_TARGET}" PROPERTIES
      MUJOCO_SIM_CONFIG_HEADER "${generated_header}"
      MUJOCO_SIM_CONFIG_SOURCE "${generated_source}"
    )
  endif()
endfunction()
