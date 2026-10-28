# SPDX-License-Identifier: GPL-3.0-only
# Benchmark registration is separate from the default correctness CTest tree.
set(DSP56300_BENCH_FILTER "" CACHE STRING "Fixture-set regex; empty selects all")
set(DSP56300_BENCH_CASES "sample" CACHE STRING "sample, all, or a zero-based case index")
set(DSP56300_BENCH_BACKEND "jit" CACHE STRING "jit, jit-scalar, jit-avx2, jit-unbatched, cached, reference, or all")
set(DSP56300_BENCH_REPLAYS "auto" CACHE STRING "auto or calls per trial in [1,1048576]")
set_property(CACHE DSP56300_BENCH_CASES PROPERTY STRINGS sample all)
set_property(CACHE DSP56300_BENCH_BACKEND PROPERTY STRINGS jit jit-scalar jit-avx2 jit-unbatched cached reference all)
if(NOT DSP56300_BENCH_BACKEND MATCHES "^(jit|jit-scalar|jit-avx2|jit-unbatched|cached|reference|all)$")
  message(FATAL_ERROR "Unknown benchmark backend: ${DSP56300_BENCH_BACKEND}")
endif()
if(NOT DSP56300_BENCH_CASES MATCHES "^(sample|all|[0-9]+)$")
  message(FATAL_ERROR "Benchmark cases must be sample, all, or one numeric index")
endif()
if(NOT DSP56300_BENCH_REPLAYS STREQUAL "auto")
  if(NOT DSP56300_BENCH_REPLAYS MATCHES "^[0-9]+$")
    message(FATAL_ERROR "Benchmark replays must be auto or a positive integer")
  endif()
  if(DSP56300_BENCH_REPLAYS LESS 1 OR DSP56300_BENCH_REPLAYS GREATER 1048576)
    message(FATAL_ERROR "Benchmark replays must be in [1,1048576]")
  endif()
endif()
file(GLOB bench_fixtures CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/*.d56")
set(bench_directory "${CMAKE_CURRENT_BINARY_DIR}/bench")
file(MAKE_DIRECTORY "${bench_directory}")
set(bench_tests "# Generated independent fixture benchmarks.\n")
foreach(fixture IN LISTS bench_fixtures)
  get_filename_component(name "${fixture}" NAME_WE)
  string(APPEND bench_tests
    "add_test([==[${name}]==] [==[$<TARGET_FILE:dsp56300_fixture_bench>]==] [==[${fixture}]==] [==[${DSP56300_BENCH_CASES}]==] [==[${DSP56300_BENCH_BACKEND}]==] [==[${DSP56300_BENCH_REPLAYS}]==])\n"
    "set_tests_properties([==[${name}]==] PROPERTIES SKIP_RETURN_CODE 77)\n")
endforeach()
file(GENERATE OUTPUT "${bench_directory}/CTestTestfile.cmake" CONTENT "${bench_tests}")
set(bench_selection)
if(NOT DSP56300_BENCH_FILTER STREQUAL "")
  list(APPEND bench_selection -R "${DSP56300_BENCH_FILTER}")
endif()
add_custom_target(bench
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${bench_directory}"
    --parallel 1 --no-tests=error --verbose ${bench_selection}
  DEPENDS dsp56300_fixture_bench
  USES_TERMINAL
  VERBATIM)
