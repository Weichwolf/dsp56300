# SPDX-License-Identifier: GPL-3.0-only
execute_process(COMMAND "${GENERATOR}" "${OUTPUT}"
  RESULT_VARIABLE generation_result OUTPUT_VARIABLE generation_output
  ERROR_VARIABLE generation_error)
if(NOT generation_result EQUAL 0)
  message(FATAL_ERROR "Fixture generator failed: ${generation_output}${generation_error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUTPUT}" "${FIXTURE}"
  RESULT_VARIABLE comparison_result)
if(NOT comparison_result EQUAL 0)
  message(FATAL_ERROR "Generated fixture differs from the reviewed binary: ${FIXTURE}")
endif()
