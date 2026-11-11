# SPDX-License-Identifier: GPL-3.0-only
# Native helpers and compiler templates use the same own ISA implementation.
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
  COMMAND "${DSP56300_LLVM_LINK}" $<TARGET_OBJECTS:${DSP56300_SEMANTIC_TARGET}> $<TARGET_OBJECTS:dsp56300_io_ir>
    -o "${DSP56300_SEMANTIC_IR}.raw"
  COMMAND "${DSP56300_LLVM_OPT}" "-passes=default<O3>,verify"
    "${DSP56300_SEMANTIC_IR}.raw" -o "${DSP56300_SEMANTIC_IR}.tmp"
  COMMAND "${CMAKE_COMMAND}" -E rename
    "${DSP56300_SEMANTIC_IR}.tmp" "${DSP56300_SEMANTIC_IR}"
  DEPENDS ${DSP56300_SEMANTIC_TARGET} dsp56300_io_ir
    $<TARGET_OBJECTS:${DSP56300_SEMANTIC_TARGET}> $<TARGET_OBJECTS:dsp56300_io_ir>
  COMMAND_EXPAND_LISTS VERBATIM)
add_custom_target(dsp56300-semantic-ir DEPENDS "${DSP56300_SEMANTIC_IR}")
