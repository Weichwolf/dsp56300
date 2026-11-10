# SPDX-License-Identifier: GPL-3.0-only
# Use the installed development package; never bootstrap LLVM from source.
find_package(LLVM 19.1 REQUIRED CONFIG
  HINTS "${DSP56300_LLVM_BIN}/../lib/cmake/llvm")
if(NOT LLVM_VERSION_MAJOR EQUAL 19 OR NOT TARGET LLVM)
  message(FATAL_ERROR "The packaged LLVM 19 shared library and headers are required")
endif()
target_include_directories(dsp56300_core SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(dsp56300_core PRIVATE LLVM)
