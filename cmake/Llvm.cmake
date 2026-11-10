# SPDX-License-Identifier: GPL-3.0-only
# Use the installed development package; never bootstrap LLVM from source.
find_program(DSP56300_LLVM_CONFIG NAMES llvm-config-19 llvm-config
  HINTS "${DSP56300_LLVM_BIN}" REQUIRED)
execute_process(COMMAND "${DSP56300_LLVM_CONFIG}" --cmakedir
  OUTPUT_VARIABLE dsp56300_llvm_cmake OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
find_package(LLVM 19.1 CONFIG REQUIRED GLOBAL
  PATHS "${dsp56300_llvm_cmake}" NO_DEFAULT_PATH)
if(NOT LLVM_VERSION_MAJOR EQUAL 19 OR NOT TARGET LLVM)
  message(FATAL_ERROR "The packaged LLVM 19 shared library and headers are required")
endif()
target_include_directories(dsp56300_core SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(dsp56300_core PRIVATE LLVM)
