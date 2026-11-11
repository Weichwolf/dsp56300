# SPDX-License-Identifier: GPL-3.0-only
find_package(Python3 REQUIRED COMPONENTS Interpreter)
file(GLOB_RECURSE dsp56300_compiler_inputs CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.cmake")
list(APPEND dsp56300_compiler_inputs
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_semantic.py")
set(dsp56300_source_list "${CMAKE_CURRENT_BINARY_DIR}/semantic/sources.txt")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/semantic")
string(JOIN "\n" dsp56300_source_names ${dsp56300_compiler_inputs})
file(GENERATE OUTPUT "${dsp56300_source_list}" CONTENT "${dsp56300_source_names}\n")
set(dsp56300_semantic_data "${CMAKE_CURRENT_BINARY_DIR}/semantic/data.cpp")
add_custom_command(OUTPUT "${dsp56300_semantic_data}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_semantic.py"
    "${DSP56300_SEMANTIC_IR}" "${dsp56300_source_list}" "${CMAKE_CURRENT_SOURCE_DIR}"
    "${dsp56300_semantic_data}" "${CMAKE_CURRENT_BINARY_DIR}/semantic/manifest.json"
    "${DSP56300_CLANG_FORMAT}"
  DEPENDS "${DSP56300_SEMANTIC_IR}" "${dsp56300_source_list}" ${dsp56300_compiler_inputs}
  VERBATIM)
target_sources(dsp56300_core PRIVATE "${dsp56300_semantic_data}")
