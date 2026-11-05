# SPDX-License-Identifier: GPL-3.0-only
cmake_minimum_required(VERSION 3.25)
project(dsp56300_core VERSION 0.1.0 LANGUAGES C CXX)

include(cmake/CompilerCache.cmake)
include(CTest)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
foreach(language C CXX)
  if(NOT CMAKE_${language}_COMPILER_ID STREQUAL "Clang" OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_GREATER_EQUAL 19 OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_LESS 20)
    message(FATAL_ERROR "Use LLVM 19: cmake --preset strict")
  endif()
endforeach()
include(cmake/Quality.cmake)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
find_program(DSP56300_CLANG_TIDY NAMES clang-tidy-19 REQUIRED)
find_program(DSP56300_CLANG_FORMAT NAMES clang-format-19 REQUIRED)
option(DSP56300_BUILD_TESTING "Build DSP library tests" ${PROJECT_IS_TOP_LEVEL})

set(CMAKE_CXX_CLANG_TIDY "${DSP56300_CLANG_TIDY};--warnings-as-errors=*")

add_library(dsp56300_core STATIC
  src/core.cpp src/core/captured.cpp src/core/bounded.cpp src/core/memory_windows.cpp src/core/instruction_boundary.cpp src/core/external_memory.cpp src/core/core_state.cpp src/core/execution_state.cpp
  src/core/staged.cpp src/core/staged_compute.cpp src/core/staged_plan.cpp src/core/move_publication.cpp src/core/move_storage.cpp src/core/alu_publication.cpp src/core/unit_publication.cpp src/core/pcu_publication.cpp src/core/hidden_retirement.cpp src/core/native_retirement.cpp src/core/program_capture.cpp src/core/instruction_shape.cpp
  src/core/transfer_info.cpp src/core/transfer_parallel.cpp src/core/transfer_special.cpp
  src/core/transfer_description.cpp src/core/arithmetic.cpp src/core/saturation.cpp src/core/shifts.cpp
  src/core/conditional.cpp src/core/immediate.cpp src/core/integer.cpp src/core/decode.cpp src/core/moves.cpp
  src/core/transfers.cpp src/core/registers.cpp src/core/agu.cpp src/core/control.cpp src/core/exceptions.cpp
  src/core/bits.cpp src/core/bit_fields.cpp src/core/loops.cpp src/core/products.cpp src/core/iteration.cpp
  src/core/processing.cpp src/core/debugging.cpp src/core/viterbi.cpp src/core/program_cache.cpp src/core/cache_control.cpp
  src/jit.cpp src/jit/jit_helpers.cpp src/jit/x64/jit_emitter.cpp src/jit/x64/x64.cpp
  src/jit/x64/jit_addition.cpp src/jit/x64/jit_logic.cpp
  src/jit/x64/jit_saturation.cpp
  src/jit/x64/jit_loops.cpp
  src/jit/x64/jit_flags.cpp
  src/jit/native_moves.cpp src/jit/native_region.cpp src/jit/dataflow.cpp src/jit/value_ranges.cpp src/jit/word_reuse.cpp src/jit/x64/jit_moves.cpp src/jit/x64/jit_windows.cpp
  src/jit/x64/jit_regions.cpp
  src/jit/x64/region_guards.cpp src/jit/x64/jit_window.cpp
  src/jit/x64/jit_word_reuse.cpp
  src/jit/x64/jit_move_address.cpp src/jit/x64/jit_move_register.cpp src/jit/x64/jit_long_moves.cpp
  src/core/memory.cpp src/jit/x64/jit_retirement.cpp src/jit/x64/jit_status.cpp
  src/jit/x64/jit_shifts.cpp src/jit/native_register_moves.cpp src/jit/x64/jit_inputs.cpp
  src/jit/mac_batch.cpp src/jit/mac_batch_scalar.cpp src/jit/x64/mac_batch_avx2.cpp src/jit/x64/jit_batch.cpp)
add_library(dsp56300::core ALIAS dsp56300_core)
target_include_directories(dsp56300_core PUBLIC include)
target_compile_features(dsp56300_core PUBLIC cxx_std_23)
target_link_libraries(dsp56300_core PUBLIC dsp56300_quality)
target_compile_options(dsp56300_core PUBLIC $<$<CONFIG:Release>:-march=native>)
target_link_options(dsp56300_core PUBLIC -fuse-ld=lld)
target_include_directories(dsp56300_core PRIVATE src/core src/jit src/jit/x64)

file(GLOB_RECURSE DSP56300_OWN_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/*.hpp")
if(PROJECT_IS_TOP_LEVEL)
  set(DSP56300_FORMAT_TARGET format-check)
else()
  set(DSP56300_FORMAT_TARGET dsp56300-format-check)
endif()
add_custom_target(${DSP56300_FORMAT_TARGET}
  COMMAND "${DSP56300_CLANG_FORMAT}" --dry-run --Werror ${DSP56300_OWN_FILES}
  VERBATIM)

if(BUILD_TESTING AND DSP56300_BUILD_TESTING)
  add_executable(dsp56300_memory_api_test test/core/memory_api_test.cpp)
  target_link_libraries(dsp56300_memory_api_test PRIVATE dsp56300::core)
  add_test(NAME memory-api COMMAND dsp56300_memory_api_test)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    add_executable(dsp56300_memory_windows_bench test/bench/memory_windows_bench.cpp)
    target_link_libraries(dsp56300_memory_windows_bench PRIVATE dsp56300::core)
    add_custom_target(bench-memory-windows
      COMMAND $<TARGET_FILE:dsp56300_memory_windows_bench> all
      DEPENDS dsp56300_memory_windows_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_memory_windows_test test/core/memory_windows_test.cpp)
    target_link_libraries(dsp56300_memory_windows_test PRIVATE dsp56300::core)
    add_test(NAME external-native-windows COMMAND dsp56300_memory_windows_test)
    add_executable(dsp56300_native_prefix_test test/jit/native_prefix_test.cpp)
    target_link_libraries(dsp56300_native_prefix_test PRIVATE dsp56300::core)
    add_test(NAME native-cached-prefixes COMMAND dsp56300_native_prefix_test)
    add_executable(dsp56300_native_repeat_bench test/bench/native_repeat_bench.cpp)
    target_link_libraries(dsp56300_native_repeat_bench PRIVATE dsp56300::core)
    add_custom_target(bench-native-repeat
      COMMAND $<TARGET_FILE:dsp56300_native_repeat_bench> all
      DEPENDS dsp56300_native_repeat_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_native_repeat_test test/jit/native_repeat_test.cpp)
    target_link_libraries(dsp56300_native_repeat_test PRIVATE dsp56300::core)
    add_test(NAME native-repeat-bodies COMMAND dsp56300_native_repeat_test)
    add_executable(dsp56300_cache_eligibility_test test/jit/native_cache_eligibility_test.cpp)
    target_link_libraries(dsp56300_cache_eligibility_test PRIVATE dsp56300::core)
    add_test(NAME native-empty-cache-eligibility COMMAND dsp56300_cache_eligibility_test)
    add_executable(dsp56300_bounded_run_test test/jit/bounded_run_test.cpp)
    target_link_libraries(dsp56300_bounded_run_test PRIVATE dsp56300::core)
    add_test(NAME bounded-native-prefixes COMMAND dsp56300_bounded_run_test)
    add_executable(dsp56300_automatic_jit_test test/jit/automatic_jit_test.cpp)
    target_link_libraries(dsp56300_automatic_jit_test PRIVATE dsp56300::core)
    add_test(NAME automatic-native-external-memory COMMAND dsp56300_automatic_jit_test)
    add_executable(dsp56300_automatic_io_bench test/bench/automatic_io_bench.cpp)
    target_link_libraries(dsp56300_automatic_io_bench PRIVATE dsp56300::core)
    add_executable(dsp56300_native_retirement_test test/jit/native_retirement_test.cpp
      test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_retirement_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_retirement_test PRIVATE
      src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-retirement-boundaries COMMAND dsp56300_native_retirement_test)
    add_executable(dsp56300_native_inputs_test test/jit/native_inputs_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_inputs_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_inputs_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-input-snapshots COMMAND dsp56300_native_inputs_test)
    add_executable(dsp56300_native_address_test test/jit/native_address_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_address_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_address_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-address-updates COMMAND dsp56300_native_address_test)
    add_executable(dsp56300_native_immediate_test test/jit/native_immediate_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_immediate_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_immediate_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-address-immediates COMMAND dsp56300_native_immediate_test)
    add_executable(dsp56300_native_loop_setup_test test/jit/native_loop_setup_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_loop_setup_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_loop_setup_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-loop-setup COMMAND dsp56300_native_loop_setup_test)
    add_executable(dsp56300_native_region_test test/jit/native_region_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_region_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-ram-regions COMMAND dsp56300_native_region_test)
    add_executable(dsp56300_native_region_constant_test test/jit/native_region_constant_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_region_constant_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_constant_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-constant-regions COMMAND dsp56300_native_region_constant_test)
    add_executable(dsp56300_native_accounting_test test/jit/native_accounting_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_accounting_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_accounting_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-region-accounting COMMAND dsp56300_native_accounting_test)
    add_executable(dsp56300_value_range_test test/jit/value_range_test.cpp)
    target_link_libraries(dsp56300_value_range_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_value_range_test PRIVATE src/core src/jit test/support)
    add_test(NAME native-value-ranges COMMAND dsp56300_value_range_test)
    add_executable(dsp56300_fused_rounding_test test/isa/fused_rounding_test.cpp)
    target_link_libraries(dsp56300_fused_rounding_test PRIVATE dsp56300::core)
    add_test(NAME native-fused-product-flags COMMAND dsp56300_fused_rounding_test)
    add_executable(dsp56300_native_product_test test/jit/native_product_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_product_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_product_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-product-coordinates COMMAND dsp56300_native_product_test)
    add_executable(dsp56300_word_reuse_test test/jit/word_reuse_test.cpp)
    target_link_libraries(dsp56300_word_reuse_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_word_reuse_test PRIVATE src/core src/jit test/support)
    add_test(NAME native-word-reuse COMMAND dsp56300_word_reuse_test)
    add_executable(dsp56300_loop_window_test test/jit/loop_window_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_loop_window_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_loop_window_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-loop-window COMMAND dsp56300_loop_window_test)
    add_executable(dsp56300_x64_immediate_test test/jit/x64_immediate_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_x64_immediate_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_x64_immediate_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-immediate-encodings COMMAND dsp56300_x64_immediate_test)
    add_executable(dsp56300_native_saturation_test test/jit/native_saturation_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_saturation_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_saturation_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME native-saturation-boundaries COMMAND dsp56300_native_saturation_test)
    add_executable(dsp56300_mac_batch_test test/isa/mac_batch_test.cpp)
    target_link_libraries(dsp56300_mac_batch_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_mac_batch_test PRIVATE src/core src/jit src/jit/x64 test/support)
    add_test(NAME guarded-mac-batches COMMAND dsp56300_mac_batch_test)
    add_executable(dsp56300_mac_batch_boundary_test test/isa/mac_batch_boundary_test.cpp)
    target_link_libraries(dsp56300_mac_batch_boundary_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_mac_batch_boundary_test PRIVATE test/support)
    add_test(NAME mac-batch-observation-boundaries COMMAND dsp56300_mac_batch_boundary_test)
    add_executable(dsp56300_native_status_test test/jit/native_status_test.cpp)
    target_link_libraries(dsp56300_native_status_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_status_test PRIVATE test/support)
    add_test(NAME native-status-boundaries COMMAND dsp56300_native_status_test)
    add_executable(dsp56300_native_shift_test test/jit/native_shift_test.cpp)
    target_link_libraries(dsp56300_native_shift_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_shift_test PRIVATE test/support)
    add_test(NAME native-shift-boundaries COMMAND dsp56300_native_shift_test)
    add_library(dsp56300_native_long_store_fixture STATIC test/support/native_long_store_fixture.cpp)
    target_link_libraries(dsp56300_native_long_store_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_native_long_store_test test/jit/native_long_store_test.cpp
      test/jit/native_long_store_boundaries.cpp)
    target_link_libraries(dsp56300_native_long_store_test PRIVATE dsp56300_native_long_store_fixture)
    add_test(NAME native-long-stores COMMAND dsp56300_native_long_store_test)
    add_library(dsp56300_native_long_load_fixture STATIC test/support/native_long_load_fixture.cpp
      test/jit/native_long_load_workload.cpp)
    target_link_libraries(dsp56300_native_long_load_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_captured_test test/core/captured_test.cpp)
    target_link_libraries(dsp56300_captured_test PRIVATE dsp56300::core)
    add_test(NAME captured-instruction COMMAND dsp56300_captured_test)
    add_executable(dsp56300_native_long_load_test test/jit/native_long_load_test.cpp
      test/jit/native_long_load_boundaries.cpp)
    target_link_libraries(dsp56300_native_long_load_test PRIVATE dsp56300_native_long_load_fixture)
    add_test(NAME native-long-loads COMMAND dsp56300_native_long_load_test)
    add_executable(dsp56300_native_long_load_bench test/bench/native_long_load_bench.cpp)
    target_link_libraries(dsp56300_native_long_load_bench PRIVATE dsp56300_native_long_load_fixture)
    add_custom_target(bench-native-long-loads
      COMMAND $<TARGET_FILE:dsp56300_native_long_load_bench> all
      DEPENDS dsp56300_native_long_load_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_native_long_store_bench test/bench/native_long_store_bench.cpp)
    target_link_libraries(dsp56300_native_long_store_bench PRIVATE dsp56300_native_long_store_fixture)
    add_custom_target(bench-native-long-stores
      COMMAND $<TARGET_FILE:dsp56300_native_long_store_bench> all
      DEPENDS dsp56300_native_long_store_bench USES_TERMINAL VERBATIM)
    add_library(dsp56300_native_partial_store_fixture STATIC test/support/native_partial_store_fixture.cpp)
    target_link_libraries(dsp56300_native_partial_store_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_native_partial_store_test test/jit/native_partial_store_test.cpp)
    target_link_libraries(dsp56300_native_partial_store_test PRIVATE dsp56300_native_partial_store_fixture)
    add_test(NAME native-partial-stores COMMAND dsp56300_native_partial_store_test)
    add_executable(dsp56300_native_partial_store_bench test/bench/native_partial_store_bench.cpp)
    target_link_libraries(dsp56300_native_partial_store_bench PRIVATE dsp56300_native_partial_store_fixture)
    add_custom_target(bench-native-partial-stores
      COMMAND $<TARGET_FILE:dsp56300_native_partial_store_bench> all
      DEPENDS dsp56300_native_partial_store_bench USES_TERMINAL VERBATIM)
  endif()
  add_executable(dsp56300_loop_control_test test/isa/loop_control_test.cpp)
  target_link_libraries(dsp56300_loop_control_test PRIVATE dsp56300::core)
  add_test(NAME loop-control-boundaries COMMAND dsp56300_loop_control_test)
  add_executable(dsp56300_cache_control_test test/core/cache_control_test.cpp)
  target_link_libraries(dsp56300_cache_control_test PRIVATE dsp56300::core)
  add_test(NAME architectural-cache-integration COMMAND dsp56300_cache_control_test)
  add_executable(dsp56300_cache_storage_test test/core/cache_storage_test.cpp)
  target_link_libraries(dsp56300_cache_storage_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_cache_storage_test PRIVATE src/core)
  add_test(NAME architectural-cache-storage COMMAND dsp56300_cache_storage_test)
  add_executable(dsp56300_debug_test test/core/debug_test.cpp)
  target_link_libraries(dsp56300_debug_test PRIVATE dsp56300::core)
  add_test(NAME debug-boundaries COMMAND dsp56300_debug_test)
  add_executable(dsp56300_processing_test test/core/processing_test.cpp)
  target_link_libraries(dsp56300_processing_test PRIVATE dsp56300::core)
  add_test(NAME processing-boundaries COMMAND dsp56300_processing_test)
  add_library(dsp56300_external_run_fixture STATIC test/support/external_run_fixture.cpp)
  target_link_libraries(dsp56300_external_run_fixture PUBLIC dsp56300::core)
  add_executable(dsp56300_external_run_test test/core/external_run_test.cpp
    test/core/external_run_cache.cpp test/core/external_run_continuation.cpp)
  target_link_libraries(dsp56300_external_run_test PRIVATE dsp56300_external_run_fixture)
  add_test(NAME external-memory-execution COMMAND dsp56300_external_run_test)
  add_executable(dsp56300_external_run_bench test/bench/external_run_bench.cpp)
  target_link_libraries(dsp56300_external_run_bench PRIVATE dsp56300_external_run_fixture)
  add_executable(dsp56300_staged_test test/core/staged_test.cpp test/core/staged_control.cpp
    test/core/staged_effects.cpp test/core/staged_events.cpp test/core/staged_timing.cpp)
  target_link_libraries(dsp56300_staged_test PRIVATE dsp56300::core)
  add_test(NAME staged-instruction-boundaries COMMAND dsp56300_staged_test)
  add_executable(dsp56300_address_publication_test test/core/address_publication_test.cpp
    test/core/address_publication_restore.cpp)
  target_link_libraries(dsp56300_address_publication_test PRIVATE dsp56300::core)
  add_test(NAME native-address-publication COMMAND dsp56300_address_publication_test)
  add_executable(dsp56300_program_fetch_test test/core/program_fetch_test.cpp)
  target_link_libraries(dsp56300_program_fetch_test PRIVATE dsp56300::core)
  add_test(NAME native-program-fetch COMMAND dsp56300_program_fetch_test)
  add_executable(dsp56300_pcu_publication_test test/core/pcu_publication_test.cpp)
  target_link_libraries(dsp56300_pcu_publication_test PRIVATE dsp56300::core)
  add_test(NAME native-pcu-publication COMMAND dsp56300_pcu_publication_test)
  add_executable(dsp56300_pure_pcu_test test/core/pure_pcu_test.cpp)
  target_link_libraries(dsp56300_pure_pcu_test PRIVATE dsp56300::core)
  add_test(NAME captured-pure-control-publication COMMAND dsp56300_pure_pcu_test)
  add_executable(dsp56300_cpu_continuation_test test/core/cpu_continuation_test.cpp)
  target_link_libraries(dsp56300_cpu_continuation_test PRIVATE dsp56300::core)
  add_test(NAME bounded-cpu-continuations COMMAND dsp56300_cpu_continuation_test)
  add_executable(dsp56300_cacheless_plan_test test/core/cacheless_plan_test.cpp)
  target_link_libraries(dsp56300_cacheless_plan_test PRIVATE dsp56300_external_run_fixture)
  add_test(NAME captured-cache-mode-planning COMMAND dsp56300_cacheless_plan_test)
  add_executable(dsp56300_cpu_plan_storage_test test/isa/cpu_plan_storage_test.cpp)
  target_include_directories(dsp56300_cpu_plan_storage_test PRIVATE src/core)
  target_link_libraries(dsp56300_cpu_plan_storage_test PRIVATE dsp56300::core)
  add_test(NAME bounded-cpu-plan-storage COMMAND dsp56300_cpu_plan_storage_test)
  add_executable(dsp56300_instruction_outcome_test test/core/instruction_outcome_test.cpp)
  target_link_libraries(dsp56300_instruction_outcome_test PRIVATE dsp56300::core)
  add_test(NAME native-instruction-outcome COMMAND dsp56300_instruction_outcome_test)
  add_executable(dsp56300_instruction_outcome_io_test test/core/instruction_outcome_io_test.cpp)
  target_link_libraries(dsp56300_instruction_outcome_io_test PRIVATE dsp56300::core)
  add_test(NAME native-instruction-outcome-io COMMAND dsp56300_instruction_outcome_io_test)
  add_executable(dsp56300_vector_fetch_test test/core/vector_fetch_test.cpp)
  target_link_libraries(dsp56300_vector_fetch_test PRIVATE dsp56300::core)
  add_test(NAME native-vector-fetch COMMAND dsp56300_vector_fetch_test)
  add_executable(dsp56300_vector_restriction_test test/core/vector_restriction_test.cpp)
  target_link_libraries(dsp56300_vector_restriction_test PRIVATE dsp56300::core)
  add_test(NAME native-vector-restrictions COMMAND dsp56300_vector_restriction_test)
  add_executable(dsp56300_deferred_interrupt_test test/core/deferred_interrupt_test.cpp)
  target_link_libraries(dsp56300_deferred_interrupt_test PRIVATE dsp56300::core)
  add_test(NAME deferred-state-interrupt COMMAND dsp56300_deferred_interrupt_test)
  add_executable(dsp56300_vector_movep_test test/core/vector_movep_test.cpp)
  target_link_libraries(dsp56300_vector_movep_test PRIVATE dsp56300::core)
  add_test(NAME native-vector-movep COMMAND dsp56300_vector_movep_test)
  add_executable(dsp56300_hidden_retirement_test test/jit/hidden_retirement_test.cpp)
  target_include_directories(dsp56300_hidden_retirement_test PRIVATE src/core)
  target_link_libraries(dsp56300_hidden_retirement_test PRIVATE dsp56300::core)
  add_test(NAME hidden-retirement-merge COMMAND dsp56300_hidden_retirement_test)
  add_executable(dsp56300_unit_publication_test test/core/unit_publication_test.cpp
    test/core/unit_publication_model.cpp test/core/unit_publication_bits.cpp test/core/unit_publication_boundaries.cpp
    test/core/unit_publication_ram.cpp)
  target_link_libraries(dsp56300_unit_publication_test PRIVATE dsp56300::core)
  add_test(NAME native-unit-publication COMMAND dsp56300_unit_publication_test)
  add_executable(dsp56300_alu_publication_test test/core/alu_publication_test.cpp
    test/core/alu_publication_model.cpp test/core/alu_publication_control.cpp)
  target_link_libraries(dsp56300_alu_publication_test PRIVATE dsp56300::core)
  add_test(NAME native-alu-publication COMMAND dsp56300_alu_publication_test)
  add_executable(dsp56300_move_publication_test test/core/move_publication_test.cpp
    test/core/move_publication_effects.cpp test/core/move_publication_restore.cpp)
  target_link_libraries(dsp56300_move_publication_test PRIVATE dsp56300::core)
  add_test(NAME native-move-publication COMMAND dsp56300_move_publication_test)
  add_executable(dsp56300_partial_accumulator_test test/isa/partial_accumulator_test.cpp)
  target_link_libraries(dsp56300_partial_accumulator_test PRIVATE dsp56300::core)
  add_test(NAME partial-accumulator-moves COMMAND dsp56300_partial_accumulator_test)
  add_executable(dsp56300_transfer_info_test test/isa/transfer_info_test.cpp
    test/isa/transfer_info_parallel.cpp test/isa/transfer_info_special.cpp test/isa/transfer_info_literals.cpp)
  target_link_libraries(dsp56300_transfer_info_test PRIVATE dsp56300::core)
  add_test(NAME data-transfer-encodings COMMAND dsp56300_transfer_info_test)
  add_executable(dsp56300_instruction_shape_test test/core/instruction_shape_test.cpp)
  target_link_libraries(dsp56300_instruction_shape_test PRIVATE dsp56300::core)
  add_test(NAME instruction-shapes COMMAND dsp56300_instruction_shape_test)
  add_executable(dsp56300_core_test test/isa/core_test.cpp)
  target_link_libraries(dsp56300_core_test PRIVATE dsp56300::core)
  add_test(NAME core-instructions COMMAND dsp56300_core_test)
  add_executable(dsp56300_magnitude_compare_test test/isa/magnitude_compare_test.cpp)
  target_link_libraries(dsp56300_magnitude_compare_test PRIVATE dsp56300::core)
  add_test(NAME magnitude-compare-boundaries COMMAND dsp56300_magnitude_compare_test)
  add_executable(dsp56300_sm_control_test test/isa/sm_control_test.cpp)
  target_link_libraries(dsp56300_sm_control_test PRIVATE dsp56300::core)
  add_test(NAME sm-control-transfers COMMAND dsp56300_sm_control_test)
  add_executable(dsp56300_sm_shifted_test test/isa/sm_shifted_test.cpp)
  target_link_libraries(dsp56300_sm_shifted_test PRIVATE dsp56300::core)
  add_test(NAME sm-shifted-arithmetic COMMAND dsp56300_sm_shifted_test)
  add_executable(dsp56300_short_move_cycles_test test/isa/short_move_cycles_test.cpp)
  target_link_libraries(dsp56300_short_move_cycles_test PRIVATE dsp56300::core)
  add_test(NAME short-move-base-cycles COMMAND dsp56300_short_move_cycles_test)
  add_executable(dsp56300_jit_test test/jit/jit_test.cpp)
  target_link_libraries(dsp56300_jit_test PRIVATE dsp56300::core)
  add_test(NAME native-boundaries COMMAND dsp56300_jit_test)
  add_executable(dsp56300_native_move_test test/jit/native_move_test.cpp)
  target_link_libraries(dsp56300_native_move_test PRIVATE dsp56300::core)
  add_test(NAME native-move-boundaries COMMAND dsp56300_native_move_test)
  add_executable(dsp56300_native_logic_test test/jit/native_logic_test.cpp)
  target_link_libraries(dsp56300_native_logic_test PRIVATE dsp56300::core)
  add_test(NAME native-word-logic COMMAND dsp56300_native_logic_test)
  add_executable(dsp56300_native_dataflow_test test/jit/native_dataflow_test.cpp)
  target_link_libraries(dsp56300_native_dataflow_test PRIVATE dsp56300::core)
  add_test(NAME native-dataflow-boundaries COMMAND dsp56300_native_dataflow_test)
  add_executable(dsp56300_native_loop_test test/jit/native_loop_test.cpp)
  target_link_libraries(dsp56300_native_loop_test PRIVATE dsp56300::core)
  add_test(NAME native-loop-boundaries COMMAND dsp56300_native_loop_test)
  add_executable(dsp56300_native_backedge_test test/jit/native_backedge_test.cpp)
  target_link_libraries(dsp56300_native_backedge_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_backedge_test PRIVATE src/core src/jit)
  add_test(NAME native-backedge-boundaries COMMAND dsp56300_native_backedge_test)
  add_executable(dsp56300_transfer_test test/isa/transfer_test.cpp)
  target_link_libraries(dsp56300_transfer_test PRIVATE dsp56300::core)
  add_test(NAME transfer-boundaries COMMAND dsp56300_transfer_test)
  add_executable(dsp56300_displacement_test test/isa/displacement_test.cpp)
  target_link_libraries(dsp56300_displacement_test PRIVATE dsp56300::core)
  add_test(NAME displacement-boundaries COMMAND dsp56300_displacement_test)
  add_executable(dsp56300_exception_test test/isa/exception_test.cpp)
  target_link_libraries(dsp56300_exception_test PRIVATE dsp56300::core)
  add_test(NAME exception-boundaries COMMAND dsp56300_exception_test)
  add_executable(dsp56300_return_test test/isa/return_test.cpp)
  target_link_libraries(dsp56300_return_test PRIVATE dsp56300::core)
  add_test(NAME return-boundaries COMMAND dsp56300_return_test)
  add_executable(dsp56300_extended_move_test test/isa/extended_move_test.cpp)
  target_link_libraries(dsp56300_extended_move_test PRIVATE dsp56300::core)
  add_test(NAME extended-move-boundaries COMMAND dsp56300_extended_move_test)
  add_executable(dsp56300_control_byte_test test/isa/control_byte_test.cpp)
  target_link_libraries(dsp56300_control_byte_test PRIVATE dsp56300::core)
  add_test(NAME control-byte-boundaries COMMAND dsp56300_control_byte_test)
  add_executable(dsp56300_selection_test test/isa/selection_test.cpp)
  target_link_libraries(dsp56300_selection_test PRIVATE dsp56300::core)
  add_test(NAME selection-boundaries COMMAND dsp56300_selection_test)
  add_executable(dsp56300_viterbi_test test/isa/viterbi_test.cpp)
  target_link_libraries(dsp56300_viterbi_test PRIVATE dsp56300::core)
  add_test(NAME viterbi-boundaries COMMAND dsp56300_viterbi_test)
  add_executable(dsp56300_bit_access_test test/isa/bit_access_test.cpp)
  target_link_libraries(dsp56300_bit_access_test PRIVATE dsp56300::core)
  add_test(NAME bit-access-boundaries COMMAND dsp56300_bit_access_test)
  add_executable(dsp56300_addressing_test test/isa/addressing_test.cpp)
  target_link_libraries(dsp56300_addressing_test PRIVATE dsp56300::core)
  add_test(NAME addressing-boundaries COMMAND dsp56300_addressing_test)
  add_executable(dsp56300_reverse_carry_test test/isa/reverse_carry_test.cpp)
  target_link_libraries(dsp56300_reverse_carry_test PRIVATE dsp56300::core)
  add_test(NAME reverse-carry-circuit COMMAND dsp56300_reverse_carry_test)
  add_executable(dsp56300_memory_alias_test test/core/memory_alias_test.cpp)
  target_link_libraries(dsp56300_memory_alias_test PRIVATE dsp56300::core)
  add_test(NAME memory-alias-boundaries COMMAND dsp56300_memory_alias_test)
  add_library(dsp56300_fixture_format STATIC test/support/binary_fixture.cpp)
  target_include_directories(dsp56300_fixture_format PUBLIC test/support include)
  target_link_libraries(dsp56300_fixture_format PUBLIC dsp56300_quality)
  add_executable(dsp56300_fixture_format_test test/core/fixture_format_test.cpp)
  target_link_libraries(dsp56300_fixture_format_test PRIVATE dsp56300_fixture_format)
  add_test(NAME fixture-format-versions COMMAND dsp56300_fixture_format_test
    "${CMAKE_CURRENT_BINARY_DIR}/fixture-format-test.d56")
  add_executable(dsp56300_generate_arithmetic test/isa/generate_arithmetic.cpp)
  target_link_libraries(dsp56300_generate_arithmetic PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_bits test/isa/generate_bit_operations.cpp)
  target_link_libraries(dsp56300_generate_bits PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_conditions test/isa/generate_conditions.cpp)
  target_link_libraries(dsp56300_generate_conditions PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_transfers test/isa/generate_transfers.cpp)
  target_link_libraries(dsp56300_generate_transfers PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_bit_access test/isa/generate_bit_access.cpp)
  target_link_libraries(dsp56300_generate_bit_access PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_modulo_strides test/isa/generate_modulo_strides.cpp)
  target_link_libraries(dsp56300_generate_modulo_strides PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_addressing test/isa/generate_addressing.cpp)
  target_link_libraries(dsp56300_generate_addressing PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_addition test/isa/generate_addition.cpp)
  target_link_libraries(dsp56300_generate_addition PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_fft test/isa/generate_fft.cpp)
  target_link_libraries(dsp56300_generate_fft PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_parallel_moves test/isa/generate_parallel_moves.cpp)
  target_link_libraries(dsp56300_generate_parallel_moves PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_displacement test/isa/generate_displacement.cpp)
  target_link_libraries(dsp56300_generate_displacement PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_debug test/isa/generate_debug.cpp)
  target_link_libraries(dsp56300_generate_debug PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_processing test/isa/generate_processing.cpp)
  target_link_libraries(dsp56300_generate_processing PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_cache_controls test/isa/generate_cache_controls.cpp)
  target_link_libraries(dsp56300_generate_cache_controls PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_exceptions test/isa/generate_exceptions.cpp)
  target_link_libraries(dsp56300_generate_exceptions PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_returns test/isa/generate_returns.cpp)
  target_link_libraries(dsp56300_generate_returns PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_extended_moves test/isa/generate_extended_moves.cpp)
  target_link_libraries(dsp56300_generate_extended_moves PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_control_bytes test/isa/generate_control_bytes.cpp)
  target_link_libraries(dsp56300_generate_control_bytes PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_selection test/isa/generate_selection.cpp)
  target_link_libraries(dsp56300_generate_selection PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_dataflow test/isa/generate_dataflow.cpp)
  target_link_libraries(dsp56300_generate_dataflow PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_loops test/isa/generate_loops.cpp)
  target_link_libraries(dsp56300_generate_loops PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_flags test/isa/generate_flags.cpp)
  target_link_libraries(dsp56300_generate_flags PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_saturation_test test/isa/saturation_test.cpp)
  target_link_libraries(dsp56300_saturation_test PRIVATE dsp56300::core)
  add_test(NAME saturation-boundaries COMMAND dsp56300_saturation_test)
  add_executable(dsp56300_generate_saturation test/isa/generate_saturation.cpp)
  target_link_libraries(dsp56300_generate_saturation PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_saturated_shifts test/isa/generate_saturated_shifts.cpp)
  target_link_libraries(dsp56300_generate_saturated_shifts PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_saturated_shift_test test/isa/saturated_shift_test.cpp)
  target_link_libraries(dsp56300_saturated_shift_test PRIVATE dsp56300::core)
  add_test(NAME saturated-shift-boundaries COMMAND dsp56300_saturated_shift_test)
  add_executable(dsp56300_generate_saturated_normalization test/isa/generate_saturated_normalization.cpp)
  target_link_libraries(dsp56300_generate_saturated_normalization PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_saturated_normalization_test test/isa/saturated_normalization_test.cpp)
  target_link_libraries(dsp56300_saturated_normalization_test PRIVATE dsp56300::core)
  add_test(NAME saturated-normalization-boundaries COMMAND dsp56300_saturated_normalization_test)
  add_executable(dsp56300_generate_bfu_logic test/isa/generate_bfu_logic.cpp)
  target_link_libraries(dsp56300_generate_bfu_logic PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_bfu_logic_test test/isa/bfu_logic_test.cpp)
  target_link_libraries(dsp56300_bfu_logic_test PRIVATE dsp56300::core)
  add_test(NAME bfu-logic-boundaries COMMAND dsp56300_bfu_logic_test)
  add_executable(dsp56300_generate_integer test/isa/generate_integer.cpp)
  target_link_libraries(dsp56300_generate_integer PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_address_calc test/isa/generate_address_calc.cpp)
  target_link_libraries(dsp56300_generate_address_calc PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_iteration test/isa/generate_iteration.cpp)
  target_link_libraries(dsp56300_generate_iteration PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_normalization test/isa/generate_normalization.cpp)
  target_link_libraries(dsp56300_generate_normalization PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_viterbi test/isa/generate_viterbi.cpp)
  target_link_libraries(dsp56300_generate_viterbi PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_fused_rounding test/isa/generate_fused_rounding.cpp)
  target_link_libraries(dsp56300_generate_fused_rounding PRIVATE dsp56300_fixture_format)
  add_test(NAME fused-rounding-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fused-rounding.d56")
  add_test(NAME fused-rounding-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_fused_rounding>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/fused-rounding-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fused-rounding.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_executable(dsp56300_generate_products test/isa/generate_products.cpp)
  target_link_libraries(dsp56300_generate_products PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_loop_control test/isa/generate_loop_control.cpp)
  target_link_libraries(dsp56300_generate_loop_control PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_bit_fields test/isa/generate_bit_fields.cpp)
  target_link_libraries(dsp56300_generate_bit_fields PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_alu_fixture_test test/isa/alu_fixture_test.cpp)
  target_link_libraries(dsp56300_alu_fixture_test PRIVATE dsp56300::core dsp56300_fixture_format)
  foreach(archive arithmetic addition saturation iteration normalization products-immediate products-mixed fused-rounding bit-fields bfu-logic-sm bfu-shifts-sm selection)
    add_test(NAME alu-publication-${archive} COMMAND dsp56300_alu_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${archive}.d56")
  endforeach()
  foreach(archive bit-access bit-branches control-bytes address-calculations)
    add_test(NAME unit-publication-${archive} COMMAND dsp56300_alu_fixture_test --units
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${archive}.d56")
  endforeach()
  foreach(archive arithmetic addition saturation iteration normalization products-immediate products-mixed fused-rounding bit-fields bfu-logic-sm bfu-shifts-sm selection bit-access bit-branches control-bytes address-calculations)
    add_test(NAME captured-pcu-publication-${archive} COMMAND dsp56300_alu_fixture_test --captured-pcu
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${archive}.d56")
  endforeach()
  foreach(archive loops loop-control return-state return-sequences exception-state exception-sequences debug-state debug-sequences processing-state processing-sequences)
    add_test(NAME pcu-publication-${archive} COMMAND dsp56300_alu_fixture_test --pcu
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${archive}.d56")
  endforeach()
  add_executable(dsp56300_fixture_test test/isa/fixture_test.cpp)
  target_link_libraries(dsp56300_fixture_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_executable(dsp56300_fixture_bench test/bench/fixture_bench.cpp)
  add_executable(dsp56300_continuous_bench test/bench/continuous_bench.cpp)
  target_link_libraries(dsp56300_continuous_bench PRIVATE dsp56300::core dsp56300_fixture_format)
  include(cmake/Bench.cmake)
  add_executable(dsp56300_generate_fir test/isa/generate_fir.cpp)
  target_link_libraries(dsp56300_generate_fir PRIVATE dsp56300_fixture_format)
  foreach(workload fir iir spectral nonlinear)
    add_executable(dsp56300_generate_${workload}_stream test/isa/generate_${workload}_stream.cpp)
    target_link_libraries(dsp56300_generate_${workload}_stream PRIVATE dsp56300_fixture_format)
    add_test(NAME ${workload}-stream-fixtures COMMAND dsp56300_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${workload}-stream-check.d56")
    foreach(kind check full)
      if(kind STREQUAL "check")
        set(stream_fixture ${workload}-stream-check)
      else()
        set(stream_fixture ${workload}-stream)
      endif()
      add_test(NAME ${stream_fixture}-reproducibility COMMAND "${CMAKE_COMMAND}"
        "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_${workload}_stream>"
        "-DGENERATOR_ARGUMENTS=${kind}"
        "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/${stream_fixture}-regenerated.d56"
        "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${stream_fixture}.d56"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
    endforeach()
  endforeach()
  add_executable(dsp56300_generate_iir test/isa/generate_iir.cpp)
  target_link_libraries(dsp56300_generate_iir PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_iir_resume_test test/isa/iir_resume_test.cpp)
  target_link_libraries(dsp56300_iir_resume_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_test(NAME iir-resume COMMAND dsp56300_iir_resume_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/iir.d56")
  add_test(NAME iir-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/iir.d56")
  add_test(NAME iir-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_iir>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/iir-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/iir.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME fir-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir.d56")
  add_test(NAME fir-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_fir>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/fir-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  target_link_libraries(dsp56300_fixture_bench PRIVATE dsp56300::core dsp56300_fixture_format)
  find_package(Threads REQUIRED)
  add_executable(dsp56300_fft_native_bench test/bench/fft_native_bench.cpp)
  target_link_libraries(dsp56300_fft_native_bench PRIVATE dsp56300_fixture_format Threads::Threads)
  add_test(NAME fft-native-algorithm COMMAND dsp56300_fft_native_bench --verify
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fft.d56")
  add_test(NAME arithmetic-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/arithmetic.d56")
  add_test(NAME condition-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/conditions.d56")
  add_test(NAME transfer-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/transfers.d56")
  add_test(NAME bit-access-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-access.d56")
  add_test(NAME bit-branch-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-branches.d56")
  add_test(NAME addressing-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addressing.d56")
  add_test(NAME addition-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addition.d56")
  add_test(NAME fft-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fft.d56")
  add_test(NAME parallel-move-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/parallel-moves.d56")
  add_test(NAME displacement-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/displacement-short.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/displacement-long.d56")
  add_test(NAME debug-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/debug-state.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/debug-sequences.d56")
  foreach(debug_group IN ITEMS state sequences)
    add_test(NAME debug-${debug_group}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_debug>"
      "-DGENERATOR_ARGUMENTS=${debug_group}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/debug-${debug_group}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/debug-${debug_group}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME processing-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/processing-state.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/processing-sequences.d56")
  foreach(processing_group IN ITEMS state sequences)
    add_test(NAME processing-${processing_group}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_processing>"
      "-DGENERATOR_ARGUMENTS=${processing_group}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/processing-${processing_group}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/processing-${processing_group}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME cache-control-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/cache-control-state.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/cache-control-sequences.d56")
  foreach(cache_group IN ITEMS state sequences)
    add_test(NAME cache-control-${cache_group}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_cache_controls>"
      "-DGENERATOR_ARGUMENTS=${cache_group}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/cache-control-${cache_group}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/cache-control-${cache_group}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME exception-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/exception-state.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/exception-sequences.d56")
  foreach(exception_group IN ITEMS state sequences)
    add_test(NAME exception-${exception_group}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_exceptions>"
      "-DGENERATOR_ARGUMENTS=${exception_group}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/exception-${exception_group}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/exception-${exception_group}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME return-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/return-state.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/return-sequences.d56")
  foreach(return_group IN ITEMS state sequences)
    add_test(NAME return-${return_group}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_returns>"
      "-DGENERATOR_ARGUMENTS=${return_group}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/return-${return_group}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/return-${return_group}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME extended-move-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/immediate-moves.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/long-moves.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/mixed-moves.d56")
  foreach(move_form IN ITEMS immediate long mixed)
    add_test(NAME ${move_form}-move-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_extended_moves>"
      "-DGENERATOR_ARGUMENTS=${move_form}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/${move_form}-moves-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${move_form}-moves.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME selection-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/selection.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/test-complement.d56"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/control-bytes.d56")
  add_test(NAME selection-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_selection>"
    "-DGENERATOR_ARGUMENTS=select"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/selection-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/selection.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME test-complement-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_selection>"
    "-DGENERATOR_ARGUMENTS=unary"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/test-complement-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/test-complement.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME control-byte-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_control_bytes>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/control-bytes-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/control-bytes.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME displacement-short-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_displacement>"
    "-DGENERATOR_ARGUMENTS=short"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/displacement-short-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/displacement-short.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME displacement-long-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_displacement>"
    "-DGENERATOR_ARGUMENTS=long"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/displacement-long-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/displacement-long.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME native-dataflow-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/native-dataflow.d56")
  add_test(NAME sm-loop-counts-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-loop-counts.d56")
  add_test(NAME sm-loop-counts-owned-publication COMMAND dsp56300_alu_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-loop-counts.d56")
  add_test(NAME sm-loop-counts-captured-publication COMMAND dsp56300_alu_fixture_test --captured-pcu
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-loop-counts.d56")
  add_test(NAME sm-cpu-domains-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-cpu-domains.d56")
  add_test(NAME sm-cpu-domains-owned-publication COMMAND dsp56300_alu_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-cpu-domains.d56")
  add_test(NAME sm-cpu-domains-captured-publication COMMAND dsp56300_alu_fixture_test --captured-pcu
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/sm-cpu-domains.d56")
  add_test(NAME saturation-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/saturation.d56")
  add_test(NAME saturation-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_saturation>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/saturation-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/saturation.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME saturated-shift-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/saturated-shifts.d56")
  add_test(NAME saturated-shift-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_saturated_shifts>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/saturated-shifts-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/saturated-shifts.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME loop-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loops.d56")
  add_test(NAME flag-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/flags.d56")
  add_test(NAME integer-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/integer.d56")
  foreach(kind pure memory displacement)
    add_test(NAME modulo-strides-${kind}-fixtures COMMAND dsp56300_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/modulo-strides-${kind}.d56")
    add_test(NAME modulo-strides-${kind}-owned-publication COMMAND dsp56300_alu_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/modulo-strides-${kind}.d56")
    add_test(NAME modulo-strides-${kind}-captured-publication COMMAND dsp56300_alu_fixture_test --captured-pcu
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/modulo-strides-${kind}.d56")
    add_test(NAME modulo-strides-${kind}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_modulo_strides>"
      "-DGENERATOR_ARGUMENTS=${kind}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/modulo-strides-${kind}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/modulo-strides-${kind}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME address-calculation-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/address-calculations.d56")
  add_test(NAME iteration-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/iteration.d56")
  add_test(NAME normalization-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/normalization.d56")
  foreach(kind fast iteration)
    add_test(NAME saturated-normalization-${kind}-fixtures COMMAND dsp56300_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/normalization-sm-${kind}.d56")
    add_test(NAME saturated-normalization-${kind}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_saturated_normalization>"
      "-DGENERATOR_ARGUMENTS=${kind}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/normalization-sm-${kind}-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/normalization-sm-${kind}.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  foreach(kind logic shifts)
    add_test(NAME bfu-${kind}-fixtures COMMAND dsp56300_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bfu-${kind}-sm.d56")
    add_test(NAME bfu-${kind}-reproducibility COMMAND "${CMAKE_COMMAND}"
      "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bfu_logic>"
      "-DGENERATOR_ARGUMENTS=${kind}"
      "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bfu-${kind}-sm-regenerated.d56"
      "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bfu-${kind}-sm.d56"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  endforeach()
  add_test(NAME viterbi-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/viterbi.d56")
  add_test(NAME viterbi-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_viterbi>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/viterbi-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/viterbi.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME normalization-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_normalization>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/normalization-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/normalization.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME address-calculation-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_address_calc>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/address-calculations-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/address-calculations.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME iteration-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_iteration>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/iteration-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/iteration.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME immediate-product-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/products-immediate.d56")
  add_test(NAME mixed-product-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/products-mixed.d56")
  add_test(NAME product-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_products>"
    "-DOUTPUT_IMMEDIATE=${CMAKE_CURRENT_BINARY_DIR}/products-immediate-regenerated.d56"
    "-DOUTPUT_MIXED=${CMAKE_CURRENT_BINARY_DIR}/products-mixed-regenerated.d56"
    "-DFIXTURE_IMMEDIATE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/products-immediate.d56"
    "-DFIXTURE_MIXED=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/products-mixed.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_product_fixtures.cmake")
  add_test(NAME loop-control-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loop-control.d56")
  add_test(NAME loop-control-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_loop_control>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/loop-control-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loop-control.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME bit-field-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-fields.d56")
  add_test(NAME bit-field-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bit_fields>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bit-fields-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-fields.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME integer-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_integer>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/integer-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/integer.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME flag-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_flags>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/flags-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/flags.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME loop-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_loops>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/loops-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loops.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME native-dataflow-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_dataflow>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/native-dataflow-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/native-dataflow.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME parallel-move-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_parallel_moves>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/parallel-moves-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/parallel-moves.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME fft-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_fft>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/fft-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fft.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME addition-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_addition>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/addition-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addition.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME addressing-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_addressing>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/addressing-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addressing.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME bit-access-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bit_access>"
    "-DACCESS_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bit-access-regenerated.d56"
    "-DBRANCH_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bit-branches-regenerated.d56"
    "-DACCESS_FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-access.d56"
    "-DBRANCH_FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-branches.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_bit_access.cmake")
  add_test(NAME transfer-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_transfers>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/transfers-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/transfers.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME condition-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_conditions>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/conditions-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/conditions.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME bit-operation-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-operations.d56")
  add_test(NAME bit-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bits>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bits-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-operations.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  add_test(NAME fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_arithmetic>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/arithmetic-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/arithmetic.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  get_directory_property(core_test_targets BUILDSYSTEM_TARGETS)
  foreach(core_test_target IN LISTS core_test_targets)
    get_target_property(core_test_sources ${core_test_target} SOURCES)
    if(core_test_sources MATCHES "test/")
      target_include_directories(${core_test_target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test/support")
    endif()
  endforeach()
endif()
