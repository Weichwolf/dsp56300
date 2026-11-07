# SPDX-License-Identifier: GPL-3.0-only
# Only continuous workloads qualify throughput; microfixtures remain diagnostics.
set(DSP56300_BENCH_FILTER "" CACHE STRING "Continuous workload regex; empty selects all")
set(DSP56300_BENCH_CASES "all" CACHE STRING "sample, all, or a zero-based case index")
option(DSP56300_BENCH_ENFORCE_DEADLINE "Fail optimized bench on any cold or warm processing deadline miss" ON)
set_property(CACHE DSP56300_BENCH_CASES PROPERTY STRINGS sample all)
if(NOT DSP56300_BENCH_CASES MATCHES "^(sample|all|[0-9]+)$")
  message(FATAL_ERROR "Benchmark cases must be sample, all, or one numeric index")
endif()
set(deadline_policy report)
if(DSP56300_BENCH_ENFORCE_DEADLINE AND DSP56300_SANITIZER STREQUAL "none")
  set(deadline_policy enforce)
endif()
set(bench_directory "${CMAKE_CURRENT_BINARY_DIR}/bench")
file(MAKE_DIRECTORY "${bench_directory}")
# This explicit list contains qualified continuous program shapes. New scenarios
# need independent full-state fixtures and real-time workload metadata first.
set(bench_tests "# Generated continuous single-instance benchmarks.\n")
foreach(name fir iir spectral nonlinear)
  set(fixture "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${name}-stream.d56")
  string(APPEND bench_tests
    "add_test([==[${name}-stream]==] [==[$<TARGET_FILE:dsp56300_continuous_bench>]==] [==[${name}]==] [==[${fixture}]==] [==[${DSP56300_BENCH_CASES}]==] [==[jit]==] [==[${deadline_policy}]==])\n"
    "set_tests_properties([==[${name}-stream]==] PROPERTIES TIMEOUT 30)\n")
endforeach()
file(GENERATE OUTPUT "${bench_directory}/CTestTestfile.cmake" CONTENT "${bench_tests}")
set(bench_selection)
if(NOT DSP56300_BENCH_FILTER STREQUAL "")
  list(APPEND bench_selection -R "${DSP56300_BENCH_FILTER}")
endif()
add_custom_target(${DSP56300_TOOL_PREFIX}bench
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${bench_directory}"
    --parallel 1 --no-tests=error --verbose ${bench_selection}
  DEPENDS dsp56300_continuous_bench
  USES_TERMINAL
  VERBATIM)

# Authored external-bus microfixtures share full-state correctness oracles with
# external-memory-execution. They diagnose API cost, not continuous throughput.
set(DSP56300_EXTERNAL_BENCH_CASE "all" CACHE STRING "all, audio-mac, image-add, or communications-xor")
set_property(CACHE DSP56300_EXTERNAL_BENCH_CASE PROPERTY STRINGS all audio-mac image-add communications-xor)
if(NOT DSP56300_EXTERNAL_BENCH_CASE MATCHES "^(all|audio-mac|image-add|communications-xor)$")
  message(FATAL_ERROR "Unknown authored external-bus benchmark workload")
endif()
add_custom_target(${DSP56300_TOOL_PREFIX}bench-external-io
  COMMAND $<TARGET_FILE:dsp56300_external_run_bench> "${DSP56300_EXTERNAL_BENCH_CASE}"
  DEPENDS dsp56300_external_run_bench
  USES_TERMINAL
  VERBATIM)
