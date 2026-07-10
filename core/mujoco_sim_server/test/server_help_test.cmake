execute_process(
  COMMAND "${SERVER_EXECUTABLE}" --help
  RESULT_VARIABLE result
  OUTPUT_VARIABLE standard_output
  ERROR_VARIABLE standard_error
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "--help returned ${result}")
endif()
if(NOT standard_output MATCHES "MuJoCo MJCF model path")
  message(FATAL_ERROR "--help did not write the generated help to stdout")
endif()
if(NOT standard_error STREQUAL "")
  message(FATAL_ERROR "--help unexpectedly wrote to stderr: ${standard_error}")
endif()
