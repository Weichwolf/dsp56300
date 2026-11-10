# SPDX-License-Identifier: GPL-3.0-only
# The normal native archive and semantic compiler consume identical own-code objects.
find_program(DSP56300_LLVM_LINK NAMES llvm-link-19 llvm-link
  HINTS "${DSP56300_LLVM_BIN}" REQUIRED)
find_program(DSP56300_LLVM_OPT NAMES opt-19 opt
  HINTS "${DSP56300_LLVM_BIN}" REQUIRED)
foreach(tool IN ITEMS DSP56300_LLVM_LINK DSP56300_LLVM_OPT)
  execute_process(COMMAND "${${tool}}" --version OUTPUT_VARIABLE tool_version
    COMMAND_ERROR_IS_FATAL ANY)
  if(NOT tool_version MATCHES "version 19\\.")
    message(FATAL_ERROR "${tool} must be LLVM 19")
  endif()
endforeach()
set(DSP56300_SEMANTIC_IR "${CMAKE_CURRENT_BINARY_DIR}/semantic/dsp56300.bc")
add_custom_command(OUTPUT "${DSP56300_SEMANTIC_IR}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/semantic"
  COMMAND "${CMAKE_COMMAND}"
    "-DCONFIG=$<CONFIG>" "-P" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SemanticIrConfig.cmake"
  COMMAND "${DSP56300_LLVM_LINK}" $<TARGET_OBJECTS:dsp56300_semantics>
    -o "${DSP56300_SEMANTIC_IR}.tmp"
  COMMAND "${DSP56300_LLVM_OPT}" -passes=verify -disable-output "${DSP56300_SEMANTIC_IR}.tmp"
  COMMAND "${CMAKE_COMMAND}" -E rename
    "${DSP56300_SEMANTIC_IR}.tmp" "${DSP56300_SEMANTIC_IR}"
  DEPENDS dsp56300_semantics $<TARGET_OBJECTS:dsp56300_semantics>
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SemanticIrConfig.cmake"
  COMMAND_EXPAND_LISTS VERBATIM)
add_custom_target(dsp56300-semantic-ir DEPENDS "${DSP56300_SEMANTIC_IR}")
