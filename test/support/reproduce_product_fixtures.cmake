# SPDX-License-Identifier: GPL-3.0-only
execute_process(COMMAND "${GENERATOR}" "${OUTPUT_IMMEDIATE}" "${OUTPUT_MIXED}"
  RESULT_VARIABLE generated)
if(NOT generated EQUAL 0)
  message(FATAL_ERROR "Independent product fixture generation failed: ${generated}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUTPUT_IMMEDIATE}" "${FIXTURE_IMMEDIATE}"
  RESULT_VARIABLE immediate)
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUTPUT_MIXED}" "${FIXTURE_MIXED}"
  RESULT_VARIABLE mixed)
if(NOT immediate EQUAL 0 OR NOT mixed EQUAL 0)
  message(FATAL_ERROR "Generated product fixture bytes differ from checked-in inputs")
endif()
