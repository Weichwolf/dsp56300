# SPDX-License-Identifier: GPL-3.0-only
execute_process(COMMAND "${GENERATOR}" "${ACCESS_OUTPUT}" "${BRANCH_OUTPUT}"
  RESULT_VARIABLE generation_result OUTPUT_VARIABLE generation_output
  ERROR_VARIABLE generation_error)
if(NOT generation_result EQUAL 0)
  message(FATAL_ERROR "Bit fixture generator failed: ${generation_output}${generation_error}")
endif()
foreach(pair ACCESS BRANCH)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files
    "${${pair}_OUTPUT}" "${${pair}_FIXTURE}" RESULT_VARIABLE comparison_result)
  if(NOT comparison_result EQUAL 0)
    message(FATAL_ERROR "Generated bit fixture differs: ${${pair}_FIXTURE}")
  endif()
endforeach()
