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
get_filename_component(DSP56300_LLVM_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
find_program(DSP56300_CLANG_TIDY NAMES clang-tidy-19 clang-tidy
  HINTS "${DSP56300_LLVM_BIN}" REQUIRED)
find_program(DSP56300_CLANG_FORMAT NAMES clang-format-19 clang-format
  HINTS "${DSP56300_LLVM_BIN}" REQUIRED)
foreach(tool IN ITEMS DSP56300_CLANG_TIDY DSP56300_CLANG_FORMAT)
  execute_process(COMMAND "${${tool}}" --version OUTPUT_VARIABLE tool_version
    COMMAND_ERROR_IS_FATAL ANY)
  if(NOT tool_version MATCHES "version 19\\.")
    message(FATAL_ERROR "${tool} must be LLVM 19")
  endif()
endforeach()
option(DSP56300_BUILD_TESTING "Build DSP library tests" ${PROJECT_IS_TOP_LEVEL})

set(CMAKE_CXX_CLANG_TIDY "${DSP56300_CLANG_TIDY};--warnings-as-errors=*")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
  set(dsp56300_native_sources
    src/native_owner.cpp src/jit/a64/emitter.cpp src/jit/a64/guarded.cpp src/jit/a64/guarded_control.cpp
    src/jit/a64/guarded_guards.cpp src/jit/a64/guarded_memory.cpp src/jit/a64/batch.cpp
    src/jit/a64/flags.cpp src/jit/a64/arithmetic.cpp src/jit/a64/unary.cpp
    src/jit/a64/shifts.cpp src/jit/a64/fault.cpp src/jit/a64/conditions.cpp
    src/jit/a64/control.cpp src/jit/a64/branch.cpp src/jit/a64/stack.cpp
    src/jit/a64/agu.cpp src/jit/a64/selection.cpp src/jit/a64/conditional.cpp
    src/jit/a64/registers.cpp src/jit/a64/data_registers.cpp src/jit/a64/transfer.cpp
    src/jit/a64/move.cpp src/jit/a64/address.cpp src/jit/a64/memory.cpp
    src/jit/a64/memory_windows.cpp src/jit/a64/bus.cpp src/jit/a64/memory_move.cpp
    src/jit/a64/peripheral.cpp src/jit/a64/long_move.cpp src/jit/a64/paired_move.cpp
    src/jit/a64/mixed_move.cpp src/jit/a64/integer.cpp src/jit/a64/iteration.cpp
    src/jit/a64/products.cpp src/jit/a64/bit_fields.cpp src/jit/a64/viterbi.cpp
    src/jit/a64/bits.cpp src/jit/a64/bit_memory.cpp src/jit/a64/loop.cpp
    src/jit/a64/repeat.cpp src/jit/a64/events.cpp src/jit/a64/cache.cpp
    src/jit/a64/vector.cpp src/jit/a64/resident_region_emit.cpp src/jit/a64/resident_values.cpp
    src/jit/a64/resident_memory.cpp src/jit/a64/resident_compact.cpp src/jit/a64/resident_compact_memory.cpp
    src/jit/a64/resident_compact_word.cpp src/jit/a64/resident_fusion.cpp src/jit/a64/resident_saturation.cpp
    src/jit/a64/resident_interleaved.cpp src/jit/a64/code.cpp src/jit/a64/neon.cpp
    src/jit/a64/run.cpp src/jit/a64/mac_batch_neon.cpp)
else()
  set(dsp56300_native_sources
    src/jit/x64/emitter.cpp src/jit/x64/addition.cpp src/jit/x64/logic.cpp
    src/jit/x64/saturation.cpp src/jit/x64/loop.cpp src/jit/x64/control.cpp
    src/jit/x64/bits.cpp src/jit/x64/io.cpp src/jit/x64/peripheral.cpp
    src/jit/x64/flags.cpp src/jit/x64/move.cpp src/jit/x64/memory_windows.cpp
    src/jit/x64/resident_emit.cpp src/jit/x64/window_reuse.cpp src/jit/x64/region_guards.cpp
    src/jit/x64/loop_window.cpp src/jit/x64/values.cpp src/jit/x64/address.cpp
    src/jit/x64/registers.cpp src/jit/x64/long_move.cpp src/jit/x64/retirement.cpp
    src/jit/x64/status.cpp src/jit/x64/shifts.cpp src/jit/x64/inputs.cpp
    src/jit/x64/batch.cpp src/jit/x64/vector.cpp src/jit/code.cpp
    src/jit/run.cpp src/jit/x64/mac_batch_avx2.cpp src/jit/x64/resident_region_emit.cpp)
endif()
add_library(dsp56300_core STATIC
  src/core.cpp src/semantic.cpp src/captured.cpp src/repeat.cpp
  src/memory_windows.cpp src/external_memory.cpp src/state.cpp
  src/fetch.cpp src/jit/preparation.cpp src/jit/llvm/runtime.cpp
  src/jit/llvm/region.cpp src/jit/llvm/region_plan.cpp src/jit/llvm/alu.cpp src/instruction_shape.cpp src/transfer_info.cpp
  src/transfer_parallel.cpp src/transfer_special.cpp src/transfer_description.cpp
  src/arithmetic.cpp src/saturation.cpp src/shifts.cpp
  src/conditional.cpp src/immediate.cpp src/integer.cpp
  src/decode.cpp src/move.cpp src/transfer.cpp
  src/registers.cpp src/agu.cpp src/control.cpp
  src/exceptions.cpp src/bits.cpp src/bit_fields.cpp
  src/loop.cpp src/products.cpp src/iteration.cpp
  src/processing.cpp src/debugging.cpp src/viterbi.cpp
  src/program_cache.cpp src/cache_control.cpp src/jit.cpp
  src/runtime.cpp src/jit/arena.cpp src/jit/block_parse.cpp
  src/jit/block_emit.cpp src/jit/index.cpp src/jit/move.cpp
  src/jit/resident_plan.cpp src/jit/dataflow.cpp src/jit/value_ranges.cpp
  src/jit/word_reuse.cpp src/jit/registers.cpp src/jit/mac_batch.cpp
  src/jit/value_graph.cpp src/jit/resident_region_run.cpp src/jit/resident_region_plan.cpp
  ${dsp56300_native_sources})
add_library(dsp56300::core ALIAS dsp56300_core)
target_include_directories(dsp56300_core PUBLIC include)
target_compile_features(dsp56300_core PUBLIC cxx_std_23)
target_link_libraries(dsp56300_core PUBLIC dsp56300_quality)
include(cmake/Llvm.cmake)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
  target_compile_options(dsp56300_core PUBLIC $<$<CONFIG:Release>:-mcpu=native>)
else()
  target_compile_options(dsp56300_core PUBLIC $<$<CONFIG:Release>:-march=native>)
endif()
if(NOT APPLE)
  target_link_options(dsp56300_core PUBLIC -fuse-ld=lld)
endif()
target_include_directories(dsp56300_core PRIVATE src src/jit src/jit/x64)

file(GLOB_RECURSE DSP56300_OWN_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/*.hpp")
set(DSP56300_TOOL_PREFIX "")
if(NOT PROJECT_IS_TOP_LEVEL)
  set(DSP56300_TOOL_PREFIX "dsp56300-")
endif()
set(DSP56300_FORMAT_TARGET "${DSP56300_TOOL_PREFIX}format-check")
add_custom_target(${DSP56300_FORMAT_TARGET}
  COMMAND "${DSP56300_CLANG_FORMAT}" --dry-run --Werror ${DSP56300_OWN_FILES}
  VERBATIM)

if(BUILD_TESTING AND DSP56300_BUILD_TESTING)
  foreach(owner IN ITEMS a b)
    add_library(dsp56300_llvm_owner_${owner} STATIC test/support/llvm_owner_${owner}.cpp)
    target_include_directories(dsp56300_llvm_owner_${owner} PRIVATE src test/support)
    target_include_directories(dsp56300_llvm_owner_${owner} SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
    target_link_libraries(dsp56300_llvm_owner_${owner} PRIVATE dsp56300::core)
  endforeach()
  add_executable(dsp56300_llvm_runtime_test test/jit/llvm_runtime_test.cpp)
  target_include_directories(dsp56300_llvm_runtime_test SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
  target_link_libraries(dsp56300_llvm_runtime_test PRIVATE
    dsp56300_llvm_owner_a dsp56300_llvm_owner_b dsp56300::core)
  if(APPLE)
    find_library(DSP56300_EXPERIMENTAL_LIBRARY NAMES c++experimental
      HINTS "${DSP56300_LLVM_BIN}/../lib/c++" REQUIRED)
    target_compile_options(dsp56300_llvm_runtime_test PRIVATE -fexperimental-library)
    target_link_libraries(dsp56300_llvm_runtime_test PRIVATE "${DSP56300_EXPERIMENTAL_LIBRARY}")
  endif()
  add_test(NAME optimizing-jit-runtime COMMAND dsp56300_llvm_runtime_test)
  add_executable(dsp56300_preparation_graph_test test/jit/preparation_graph_test.cpp)
  target_include_directories(dsp56300_preparation_graph_test PRIVATE src/jit)
  target_link_libraries(dsp56300_preparation_graph_test PRIVATE dsp56300::core)
  add_test(NAME preparation-control-flow COMMAND dsp56300_preparation_graph_test)
  add_executable(dsp56300_preparation_test test/jit/preparation_test.cpp)
  target_link_libraries(dsp56300_preparation_test PRIVATE dsp56300::core)
  add_test(NAME optimized-preparation COMMAND dsp56300_preparation_test)
  add_executable(dsp56300_llvm_loop_test test/jit/llvm_loop_test.cpp)
  target_include_directories(dsp56300_llvm_loop_test PRIVATE src src/jit)
  target_include_directories(dsp56300_llvm_loop_test SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
  target_link_libraries(dsp56300_llvm_loop_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-memory-loops COMMAND dsp56300_llvm_loop_test)
  add_executable(dsp56300_llvm_window_test test/jit/llvm_window_test.cpp)
  target_include_directories(dsp56300_llvm_window_test PRIVATE src src/jit test/support)
  target_include_directories(dsp56300_llvm_window_test SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
  target_link_libraries(dsp56300_llvm_window_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-memory-windows COMMAND dsp56300_llvm_window_test)
  add_executable(dsp56300_llvm_region_test test/jit/llvm_region_test.cpp)
  target_include_directories(dsp56300_llvm_region_test PRIVATE src src/jit test/support)
  target_include_directories(dsp56300_llvm_region_test SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
  target_link_libraries(dsp56300_llvm_region_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-compound-regions COMMAND dsp56300_llvm_region_test)
  add_executable(dsp56300_llvm_reuse_test test/jit/llvm_reuse_test.cpp)
  target_include_directories(dsp56300_llvm_reuse_test PRIVATE test/support)
  target_link_libraries(dsp56300_llvm_reuse_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-region-reuse COMMAND dsp56300_llvm_reuse_test)
  add_executable(dsp56300_llvm_alu_test test/jit/llvm_alu_test.cpp)
  target_include_directories(dsp56300_llvm_alu_test PRIVATE src src/jit test/support)
  target_include_directories(dsp56300_llvm_alu_test SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
  target_link_libraries(dsp56300_llvm_alu_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-alu-semantics COMMAND dsp56300_llvm_alu_test)
  add_executable(dsp56300_llvm_arithmetic_loop_test test/jit/llvm_arithmetic_loop_test.cpp)
  target_link_libraries(dsp56300_llvm_arithmetic_loop_test PRIVATE dsp56300::core)
  add_test(NAME optimizing-arithmetic-loops COMMAND dsp56300_llvm_arithmetic_loop_test)
  add_executable(dsp56300_register_address_test test/jit/register_address_test.cpp)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
    target_sources(dsp56300_register_address_test PRIVATE test/support/native/direct_a64.cpp)
  else()
    target_sources(dsp56300_register_address_test PRIVATE test/support/native/direct.cpp)
  endif()
  target_include_directories(dsp56300_register_address_test PRIVATE src src/jit test/support)
  target_link_libraries(dsp56300_register_address_test PRIVATE dsp56300::core)
  add_test(NAME native-register-addresses COMMAND dsp56300_register_address_test)
  add_executable(dsp56300_executable_arena_test test/jit/executable_arena_test.cpp)
  target_include_directories(dsp56300_executable_arena_test PRIVATE src)
  target_link_libraries(dsp56300_executable_arena_test PRIVATE dsp56300::core)
  add_test(NAME executable-arena COMMAND dsp56300_executable_arena_test)
  add_executable(dsp56300_host_mac_test test/jit/host_mac_test.cpp)
  target_include_directories(dsp56300_host_mac_test PRIVATE src src/jit)
  target_link_libraries(dsp56300_host_mac_test PRIVATE dsp56300::core)
  add_test(NAME host-mac-products COMMAND dsp56300_host_mac_test)
  add_executable(dsp56300_mac_batch_test test/isa/mac_batch_test.cpp)
  target_link_libraries(dsp56300_mac_batch_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_mac_batch_test PRIVATE src src/jit test/support)
  add_test(NAME guarded-mac-batches COMMAND dsp56300_mac_batch_test)
  add_executable(dsp56300_mac_batch_boundary_test test/isa/mac_batch_boundary_test.cpp)
  target_link_libraries(dsp56300_mac_batch_boundary_test PRIVATE dsp56300::core)
  add_test(NAME mac-batch-observation-boundaries COMMAND dsp56300_mac_batch_boundary_test)
  add_executable(dsp56300_native_value_region_test test/jit/native_value_region_test.cpp)
  target_link_libraries(dsp56300_native_value_region_test PRIVATE dsp56300::core)
  add_test(NAME native-value-regions COMMAND dsp56300_native_value_region_test)
  add_executable(dsp56300_native_prefix_loop_test test/jit/native_prefix_loop_test.cpp)
  target_link_libraries(dsp56300_native_prefix_loop_test PRIVATE dsp56300::core)
  add_test(NAME native-prefix-loops COMMAND dsp56300_native_prefix_loop_test)
  add_executable(dsp56300_value_bounds_test test/jit/value_bounds_test.cpp)
  target_link_libraries(dsp56300_value_bounds_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_value_bounds_test PRIVATE src/jit)
  add_test(NAME native-value-bounds COMMAND dsp56300_value_bounds_test)
  add_executable(dsp56300_native_reduction_test test/jit/native_reduction_test.cpp)
  target_link_libraries(dsp56300_native_reduction_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_reduction_test PRIVATE src/jit)
  add_test(NAME native-reductions COMMAND dsp56300_native_reduction_test)
  add_executable(dsp56300_native_cycle_test test/jit/native_cycle_test.cpp)
  target_link_libraries(dsp56300_native_cycle_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_cycle_test PRIVATE src/jit)
  add_test(NAME native-read-cycles COMMAND dsp56300_native_cycle_test)
  add_executable(dsp56300_native_interleaved_test test/jit/native_interleaved_test.cpp)
  target_link_libraries(dsp56300_native_interleaved_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_interleaved_test PRIVATE src/jit)
  add_test(NAME native-interleaved-memory COMMAND dsp56300_native_interleaved_test)
  add_executable(dsp56300_reverse_stream_test test/jit/reverse_stream_test.cpp)
  target_link_libraries(dsp56300_reverse_stream_test PRIVATE dsp56300::core)
  add_test(NAME native-reverse-streams COMMAND dsp56300_reverse_stream_test)
  set_tests_properties(host-mac-products executable-arena PROPERTIES TIMEOUT 30)
  add_library(dsp56300_codegen_observer STATIC test/support/codegen_observer.cpp)
  target_include_directories(dsp56300_codegen_observer PUBLIC test/support)
  target_link_libraries(dsp56300_codegen_observer PUBLIC dsp56300_quality)
  # Observe real image publication independently of the arena's mapping strategy.
  set(dsp56300_append_symbol "_ZN8dsp563006detail15ExecutableArena6appendESt4spanIKhLm18446744073709551615EE")
  if(APPLE)
    set_source_files_properties(src/jit/arena.cpp PROPERTIES
      COMPILE_DEFINITIONS DSP56300_TEST_PUBLICATIONS)
    target_include_directories(dsp56300_codegen_observer PRIVATE src)
  else()
    target_link_options(dsp56300_codegen_observer INTERFACE
      "-Wl,--wrap=${dsp56300_append_symbol}"
      "-Wl,--undefined=__wrap_${dsp56300_append_symbol}")
  endif()
  add_executable(dsp56300_deadline_test test/core/deadline_test.cpp)
  target_link_libraries(dsp56300_deadline_test PRIVATE dsp56300::core dsp56300_codegen_observer)
  add_test(NAME execution-deadlines COMMAND dsp56300_deadline_test)
  add_executable(dsp56300_external_program_test test/core/external_program_test.cpp)
  target_link_libraries(dsp56300_external_program_test PRIVATE dsp56300::core dsp56300_codegen_observer)
  add_test(NAME external-program-writes COMMAND dsp56300_external_program_test)
  add_executable(dsp56300_interrupt_window_test test/core/interrupt_window_test.cpp)
  target_link_libraries(dsp56300_interrupt_window_test PRIVATE dsp56300::core dsp56300_codegen_observer)
  add_test(NAME interrupt-mask-windows COMMAND dsp56300_interrupt_window_test)
  add_executable(dsp56300_public_api_test test/core/public_api_test.cpp)
  target_link_libraries(dsp56300_public_api_test PRIVATE dsp56300::core)
  add_test(NAME public-api COMMAND dsp56300_public_api_test)
  add_executable(dsp56300_memory_api_test test/core/memory_api_test.cpp)
  target_link_libraries(dsp56300_memory_api_test PRIVATE dsp56300::core)
  add_test(NAME memory-api COMMAND dsp56300_memory_api_test)
  add_executable(dsp56300_memory_windows_test test/core/memory_windows_test.cpp)
  target_link_libraries(dsp56300_memory_windows_test PRIVATE dsp56300::core)
  add_test(NAME external-native-windows COMMAND dsp56300_memory_windows_test)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
    add_executable(dsp56300_a64_progress_test test/jit/a64_progress_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_a64_progress_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_a64_progress_test PRIVATE src src/jit test/support)
    add_test(NAME a64-progress-publication COMMAND dsp56300_a64_progress_test)
    add_executable(dsp56300_native_region_test test/jit/native_region_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_native_region_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_test PRIVATE src src/jit test/support)
    add_test(NAME native-ram-regions COMMAND dsp56300_native_region_test)
    add_executable(dsp56300_native_ring_test test/jit/native_ring_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_native_ring_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_ring_test PRIVATE src src/jit test/support)
    add_test(NAME native-masked-rings COMMAND dsp56300_native_ring_test)
    add_executable(dsp56300_native_compound_test test/jit/native_compound_test.cpp test/support/native/direct_a64.cpp test/support/native_long_load_fixture.cpp)
    target_link_libraries(dsp56300_native_compound_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_compound_test PRIVATE src src/jit test test/support)
    add_test(NAME native-compound-regions COMMAND dsp56300_native_compound_test)
    add_executable(dsp56300_native_mixed_region_test test/jit/native_mixed_region_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_native_mixed_region_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_mixed_region_test PRIVATE src src/jit test test/support)
    add_test(NAME native-mixed-regions COMMAND dsp56300_native_mixed_region_test)
    add_executable(dsp56300_native_region_constant_test test/jit/native_region_constant_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_native_region_constant_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_constant_test PRIVATE src src/jit test/support)
    add_test(NAME native-constant-regions COMMAND dsp56300_native_region_constant_test)
    add_executable(dsp56300_native_accounting_test test/jit/native_accounting_test.cpp test/support/native/direct_a64.cpp)
    target_link_libraries(dsp56300_native_accounting_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_accounting_test PRIVATE src src/jit test/support)
    add_test(NAME native-region-accounting COMMAND dsp56300_native_accounting_test)
    add_executable(dsp56300_a64_arithmetic_test test/jit/a64_arithmetic_test.cpp)
    target_include_directories(dsp56300_a64_arithmetic_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_arithmetic_test PRIVATE dsp56300::core)
    add_test(NAME a64-arithmetic-semantics COMMAND dsp56300_a64_arithmetic_test)
    add_executable(dsp56300_a64_shift_test test/jit/a64_shift_test.cpp)
    target_include_directories(dsp56300_a64_shift_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_shift_test PRIVATE dsp56300::core)
    add_test(NAME a64-shift-semantics COMMAND dsp56300_a64_shift_test)
    add_executable(dsp56300_a64_control_test test/jit/a64_control_test.cpp)
    target_include_directories(dsp56300_a64_control_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_control_test PRIVATE dsp56300::core)
    add_test(NAME a64-control-semantics COMMAND dsp56300_a64_control_test)
    add_executable(dsp56300_a64_conditional_test test/jit/a64_conditional_test.cpp)
    target_include_directories(dsp56300_a64_conditional_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_conditional_test PRIVATE dsp56300::core)
    add_test(NAME a64-conditional-semantics COMMAND dsp56300_a64_conditional_test)
    add_executable(dsp56300_a64_transfer_test test/jit/a64_transfer_test.cpp)
    target_include_directories(dsp56300_a64_transfer_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_transfer_test PRIVATE dsp56300::core)
    add_test(NAME a64-transfer-semantics COMMAND dsp56300_a64_transfer_test)
    add_executable(dsp56300_a64_address_test test/jit/a64_address_test.cpp)
    target_include_directories(dsp56300_a64_address_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_address_test PRIVATE dsp56300::core)
    add_test(NAME a64-address-semantics COMMAND dsp56300_a64_address_test)
    add_executable(dsp56300_a64_memory_fixture_test test/jit/a64_memory_fixture_test.cpp)
    target_include_directories(dsp56300_a64_memory_fixture_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_memory_fixture_test PRIVATE dsp56300::core dsp56300_fixture_format)
    add_test(NAME a64-memory-transfer-fixtures COMMAND dsp56300_a64_memory_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/transfers.d56")
    add_test(NAME a64-parallel-move-fixtures COMMAND dsp56300_a64_memory_fixture_test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/parallel-moves.d56")
    add_executable(dsp56300_a64_memory_test test/jit/a64_memory_test.cpp)
    target_include_directories(dsp56300_a64_memory_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_memory_test PRIVATE dsp56300::core)
    add_test(NAME a64-memory-semantics COMMAND dsp56300_a64_memory_test)
    add_executable(dsp56300_a64_bus_test test/jit/a64_bus_test.cpp)
    target_include_directories(dsp56300_a64_bus_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_bus_test PRIVATE dsp56300::core)
    add_test(NAME a64-bus-semantics COMMAND dsp56300_a64_bus_test)
    add_executable(dsp56300_a64_extended_move_test test/jit/a64_extended_move_test.cpp)
    target_include_directories(dsp56300_a64_extended_move_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_extended_move_test PRIVATE dsp56300::core)
    add_test(NAME a64-extended-move-semantics COMMAND dsp56300_a64_extended_move_test)
    add_executable(dsp56300_a64_paired_move_test test/jit/a64_paired_move_test.cpp)
    target_include_directories(dsp56300_a64_paired_move_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_paired_move_test PRIVATE dsp56300::core)
    add_test(NAME a64-paired-move-semantics COMMAND dsp56300_a64_paired_move_test)
    add_executable(dsp56300_a64_extended_bus_test test/jit/a64_extended_bus_test.cpp)
    target_include_directories(dsp56300_a64_extended_bus_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_extended_bus_test PRIVATE dsp56300::core)
    add_test(NAME a64-extended-bus-semantics COMMAND dsp56300_a64_extended_bus_test)
    add_executable(dsp56300_a64_integer_test test/jit/a64_integer_test.cpp)
    target_include_directories(dsp56300_a64_integer_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_integer_test PRIVATE dsp56300::core)
    add_test(NAME a64-integer-semantics COMMAND dsp56300_a64_integer_test)
    add_executable(dsp56300_a64_iteration_test test/jit/a64_iteration_test.cpp)
    target_include_directories(dsp56300_a64_iteration_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_iteration_test PRIVATE dsp56300::core)
    add_test(NAME a64-iteration-semantics COMMAND dsp56300_a64_iteration_test)
    add_executable(dsp56300_a64_product_test test/jit/a64_product_test.cpp)
    target_include_directories(dsp56300_a64_product_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_product_test PRIVATE dsp56300::core)
    add_test(NAME a64-product-semantics COMMAND dsp56300_a64_product_test)
    add_executable(dsp56300_a64_bit_field_test test/jit/a64_bit_field_test.cpp)
    target_include_directories(dsp56300_a64_bit_field_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_bit_field_test PRIVATE dsp56300::core)
    add_test(NAME a64-bit-field-semantics COMMAND dsp56300_a64_bit_field_test)
    add_executable(dsp56300_a64_viterbi_test test/jit/a64_viterbi_test.cpp)
    target_include_directories(dsp56300_a64_viterbi_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_viterbi_test PRIVATE dsp56300::core)
    add_test(NAME a64-viterbi-semantics COMMAND dsp56300_a64_viterbi_test)
    add_executable(dsp56300_a64_bit_register_test test/jit/a64_bit_register_test.cpp)
    target_include_directories(dsp56300_a64_bit_register_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_bit_register_test PRIVATE dsp56300::core)
    add_test(NAME a64-bit-register-semantics COMMAND dsp56300_a64_bit_register_test)
    add_executable(dsp56300_a64_bit_memory_test test/jit/a64_bit_memory_test.cpp)
    target_include_directories(dsp56300_a64_bit_memory_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_bit_memory_test PRIVATE dsp56300::core)
    add_test(NAME a64-bit-memory-semantics COMMAND dsp56300_a64_bit_memory_test)
    add_executable(dsp56300_a64_loop_test test/jit/a64_loop_test.cpp)
    target_include_directories(dsp56300_a64_loop_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_loop_test PRIVATE dsp56300::core)
    add_test(NAME a64-loop-semantics COMMAND dsp56300_a64_loop_test)
    add_executable(dsp56300_a64_owner_test test/core/a64_owner_test.cpp)
    target_include_directories(dsp56300_a64_owner_test PRIVATE test/support)
    target_link_libraries(dsp56300_a64_owner_test PRIVATE dsp56300::core)
    add_test(NAME a64-owner-boundaries COMMAND dsp56300_a64_owner_test)
    add_executable(dsp56300_a64_vector_test test/jit/a64_vector_test.cpp)
    target_include_directories(dsp56300_a64_vector_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_vector_test PRIVATE dsp56300::core dsp56300_codegen_observer)
    add_test(NAME a64-vector-semantics COMMAND dsp56300_a64_vector_test)
    add_executable(dsp56300_a64_factory_test test/jit/a64_factory_test.cpp)
    target_include_directories(dsp56300_a64_factory_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_factory_test PRIVATE dsp56300::core dsp56300_codegen_observer)
    add_test(NAME a64-jit-factory COMMAND dsp56300_a64_factory_test)
    add_executable(dsp56300_a64_cache_test test/jit/a64_cache_test.cpp)
    target_include_directories(dsp56300_a64_cache_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_cache_test PRIVATE dsp56300::core)
    add_test(NAME a64-cache-semantics COMMAND dsp56300_a64_cache_test)
    add_executable(dsp56300_a64_event_test test/jit/a64_event_test.cpp)
    target_include_directories(dsp56300_a64_event_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_event_test PRIVATE dsp56300::core)
    add_test(NAME a64-event-semantics COMMAND dsp56300_a64_event_test)
    add_executable(dsp56300_a64_repeat_test test/jit/a64_repeat_test.cpp)
    target_include_directories(dsp56300_a64_repeat_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_a64_repeat_test PRIVATE dsp56300::core)
    add_test(NAME a64-repeat-semantics COMMAND dsp56300_a64_repeat_test)
  endif()
  add_executable(dsp56300_a64_code_test test/jit/a64_code_test.cpp)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
    target_sources(dsp56300_a64_code_test PRIVATE src/jit/a64/code.cpp src/jit/a64/neon.cpp)
  endif()
  target_include_directories(dsp56300_a64_code_test PRIVATE src)
  target_link_libraries(dsp56300_a64_code_test PRIVATE dsp56300::core)
  add_test(NAME a64-code-emission COMMAND dsp56300_a64_code_test)
  add_executable(dsp56300_native_context_test test/jit/native_context_test.cpp)
  target_include_directories(dsp56300_native_context_test PRIVATE src src/jit test/support)
  target_link_libraries(dsp56300_native_context_test PRIVATE dsp56300::core)
  add_test(NAME native-context-routing COMMAND dsp56300_native_context_test)
  add_executable(dsp56300_native_index_test test/jit/native_index_test.cpp)
  target_include_directories(dsp56300_native_index_test PRIVATE src src/jit test/support)
  target_link_libraries(dsp56300_native_index_test PRIVATE dsp56300::core)
  add_test(NAME native-target-index COMMAND dsp56300_native_index_test)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    add_executable(dsp56300_native_bits_test test/jit/native_bits_test.cpp)
    target_include_directories(dsp56300_native_bits_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_native_bits_test PRIVATE dsp56300::core dsp56300_codegen_observer)
    add_test(NAME native-bit-memory COMMAND dsp56300_native_bits_test)
    add_executable(dsp56300_native_bit_io_test test/jit/native_bit_io_test.cpp)
    target_include_directories(dsp56300_native_bit_io_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_native_bit_io_test PRIVATE dsp56300::core dsp56300_codegen_observer)
    add_test(NAME native-bit-io COMMAND dsp56300_native_bit_io_test)
    add_executable(dsp56300_native_peripheral_test test/jit/native_peripheral_test.cpp)
    target_include_directories(dsp56300_native_peripheral_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_native_peripheral_test PRIVATE dsp56300::core)
    add_test(NAME native-peripheral-registers COMMAND dsp56300_native_peripheral_test)
    add_executable(dsp56300_native_control_test test/jit/native_control_test.cpp)
    target_include_directories(dsp56300_native_control_test PRIVATE src src/jit test/support)
    target_link_libraries(dsp56300_native_control_test PRIVATE dsp56300::core dsp56300_codegen_observer)
    add_test(NAME native-control-flow COMMAND dsp56300_native_control_test)
    add_executable(dsp56300_native_run_test test/jit/native_run_test.cpp)
    target_include_directories(dsp56300_native_run_test PRIVATE src src/jit src/jit/x64 test/support)
    target_link_libraries(dsp56300_native_run_test PRIVATE dsp56300::core)
    add_test(NAME native-linked-blocks COMMAND dsp56300_native_run_test)
    add_executable(dsp56300_x64_vector_test test/jit/x64_vector_test.cpp)
    target_include_directories(dsp56300_x64_vector_test PRIVATE src test/support)
    target_link_libraries(dsp56300_x64_vector_test PRIVATE dsp56300::core)
    add_test(NAME native-vector-encodings COMMAND dsp56300_x64_vector_test)
    add_executable(dsp56300_memory_windows_bench test/bench/memory_windows_bench.cpp)
    target_link_libraries(dsp56300_memory_windows_bench PRIVATE dsp56300::core)
    add_custom_target(${DSP56300_TOOL_PREFIX}bench-memory-windows
      COMMAND $<TARGET_FILE:dsp56300_memory_windows_bench> all
      DEPENDS dsp56300_memory_windows_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_native_prefix_test test/jit/native_prefix_test.cpp)
    target_link_libraries(dsp56300_native_prefix_test PRIVATE dsp56300::core)
    add_test(NAME native-cached-prefixes COMMAND dsp56300_native_prefix_test)
    add_executable(dsp56300_native_repeat_bench test/bench/native_repeat_bench.cpp)
    target_link_libraries(dsp56300_native_repeat_bench PRIVATE dsp56300::core)
    add_custom_target(${DSP56300_TOOL_PREFIX}bench-native-repeat
      COMMAND $<TARGET_FILE:dsp56300_native_repeat_bench> all
      DEPENDS dsp56300_native_repeat_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_native_repeat_test test/jit/native_repeat_test.cpp)
    target_link_libraries(dsp56300_native_repeat_test PRIVATE dsp56300::core)
    add_test(NAME native-repeat-bodies COMMAND dsp56300_native_repeat_test)
    add_executable(dsp56300_cache_eligibility_test test/jit/native_cache_eligibility_test.cpp)
    target_link_libraries(dsp56300_cache_eligibility_test PRIVATE dsp56300::core)
    add_test(NAME native-empty-cache-eligibility COMMAND dsp56300_cache_eligibility_test)

    add_executable(dsp56300_automatic_jit_test test/jit/automatic_jit_test.cpp)
    target_link_libraries(dsp56300_automatic_jit_test PRIVATE dsp56300::core)
    add_test(NAME automatic-native-external-memory COMMAND dsp56300_automatic_jit_test)
    add_executable(dsp56300_automatic_io_bench test/bench/automatic_io_bench.cpp)
    target_link_libraries(dsp56300_automatic_io_bench PRIVATE dsp56300::core)
    add_executable(dsp56300_native_retirement_test test/jit/native_retirement_test.cpp
      test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_retirement_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_retirement_test PRIVATE
      src src/jit src/jit/x64 test/support)
    add_test(NAME native-retirement-boundaries COMMAND dsp56300_native_retirement_test)
    add_executable(dsp56300_native_inputs_test test/jit/native_inputs_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_inputs_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_inputs_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-input-snapshots COMMAND dsp56300_native_inputs_test)
    add_executable(dsp56300_native_address_test test/jit/native_address_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_address_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_address_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-address-updates COMMAND dsp56300_native_address_test)
    add_executable(dsp56300_native_reverse_test test/jit/native_reverse_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_reverse_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_reverse_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-reverse-addressing COMMAND dsp56300_native_reverse_test)
    add_executable(dsp56300_native_address_transfer_test test/jit/native_address_transfer_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_address_transfer_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_address_transfer_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-address-transfers COMMAND dsp56300_native_address_transfer_test)
    add_executable(dsp56300_native_immediate_test test/jit/native_immediate_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_immediate_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_immediate_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-address-immediates COMMAND dsp56300_native_immediate_test)
    add_executable(dsp56300_native_loop_setup_test test/jit/native_loop_setup_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_loop_setup_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_loop_setup_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-loop-setup COMMAND dsp56300_native_loop_setup_test)
    add_executable(dsp56300_native_region_test test/jit/native_region_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_region_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-ram-regions COMMAND dsp56300_native_region_test)
    add_executable(dsp56300_native_region_constant_test test/jit/native_region_constant_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_region_constant_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_region_constant_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-constant-regions COMMAND dsp56300_native_region_constant_test)
    add_executable(dsp56300_native_accounting_test test/jit/native_accounting_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_accounting_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_accounting_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-region-accounting COMMAND dsp56300_native_accounting_test)
    add_executable(dsp56300_value_range_test test/jit/value_range_test.cpp)
    target_link_libraries(dsp56300_value_range_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_value_range_test PRIVATE src src/jit test/support)
    add_test(NAME native-value-ranges COMMAND dsp56300_value_range_test)
    add_executable(dsp56300_transfer_range_test test/jit/transfer_range_test.cpp)
    target_link_libraries(dsp56300_transfer_range_test PRIVATE dsp56300::core)
    add_test(NAME native-transfer-ranges COMMAND dsp56300_transfer_range_test)
    add_executable(dsp56300_native_transfer_run_test test/jit/native_transfer_run_test.cpp)
    target_link_libraries(dsp56300_native_transfer_run_test PRIVATE dsp56300::core)
    add_test(NAME native-transfer-runs COMMAND dsp56300_native_transfer_run_test)
    add_executable(dsp56300_growth_status_test test/jit/growth_status_test.cpp)
    target_link_libraries(dsp56300_growth_status_test PRIVATE dsp56300::core)
    add_test(NAME native-growth-status COMMAND dsp56300_growth_status_test)
    add_executable(dsp56300_fused_rounding_test test/isa/fused_rounding_test.cpp)
    target_link_libraries(dsp56300_fused_rounding_test PRIVATE dsp56300::core)
    add_test(NAME native-fused-product-flags COMMAND dsp56300_fused_rounding_test)
    add_executable(dsp56300_native_product_test test/jit/native_product_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_product_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_product_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-product-coordinates COMMAND dsp56300_native_product_test)
    add_executable(dsp56300_word_reuse_test test/jit/word_reuse_test.cpp)
    target_link_libraries(dsp56300_word_reuse_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_word_reuse_test PRIVATE src src/jit test/support)
    add_test(NAME native-word-reuse COMMAND dsp56300_word_reuse_test)
    add_executable(dsp56300_loop_window_test test/jit/loop_window_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_loop_window_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_loop_window_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-loop-window COMMAND dsp56300_loop_window_test)
    add_executable(dsp56300_x64_immediate_test test/jit/x64_immediate_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_x64_immediate_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_x64_immediate_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-immediate-encodings COMMAND dsp56300_x64_immediate_test)
    add_executable(dsp56300_native_saturation_test test/jit/native_saturation_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_saturation_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_saturation_test PRIVATE src src/jit src/jit/x64 test/support)
    add_test(NAME native-saturation-boundaries COMMAND dsp56300_native_saturation_test)
    add_executable(dsp56300_native_status_test test/jit/native_status_test.cpp)
    target_link_libraries(dsp56300_native_status_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_status_test PRIVATE test/support)
    add_test(NAME native-status-boundaries COMMAND dsp56300_native_status_test)
    add_executable(dsp56300_native_shift_test test/jit/native_shift_test.cpp)
    target_link_libraries(dsp56300_native_shift_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_shift_test PRIVATE test/support)
    add_test(NAME native-shift-boundaries COMMAND dsp56300_native_shift_test)
    add_executable(dsp56300_native_left_shift_test test/jit/native_left_shift_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_left_shift_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_left_shift_test PRIVATE src src/jit test/support)
    add_test(NAME native-left-shifts COMMAND dsp56300_native_left_shift_test)
    add_executable(dsp56300_native_immediate_shift_test test/jit/native_immediate_shift_test.cpp test/support/native/direct.cpp)
    target_link_libraries(dsp56300_native_immediate_shift_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_native_immediate_shift_test PRIVATE src src/jit test/support)
    add_test(NAME native-immediate-shifts COMMAND dsp56300_native_immediate_shift_test)
    add_library(dsp56300_native_long_store_fixture STATIC test/support/native_long_store_fixture.cpp)
    target_link_libraries(dsp56300_native_long_store_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_native_long_store_test test/jit/native_long_store_test.cpp
      test/jit/native_long_store_boundaries.cpp)
    target_link_libraries(dsp56300_native_long_store_test PRIVATE dsp56300_native_long_store_fixture)
    add_test(NAME native-long-stores COMMAND dsp56300_native_long_store_test)
    add_library(dsp56300_native_long_load_fixture STATIC test/support/native_long_load_fixture.cpp
      test/jit/native_long_load_workload.cpp)
    target_link_libraries(dsp56300_native_long_load_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_native_long_load_test test/jit/native_long_load_test.cpp
      test/jit/native_long_load_boundaries.cpp)
    target_link_libraries(dsp56300_native_long_load_test PRIVATE dsp56300_native_long_load_fixture)
    add_test(NAME native-long-loads COMMAND dsp56300_native_long_load_test)
    add_executable(dsp56300_native_long_load_bench test/bench/native_long_load_bench.cpp)
    target_link_libraries(dsp56300_native_long_load_bench PRIVATE dsp56300_native_long_load_fixture)
    add_custom_target(${DSP56300_TOOL_PREFIX}bench-native-long-loads
      COMMAND $<TARGET_FILE:dsp56300_native_long_load_bench> all
      DEPENDS dsp56300_native_long_load_bench USES_TERMINAL VERBATIM)
    add_executable(dsp56300_native_long_store_bench test/bench/native_long_store_bench.cpp)
    target_link_libraries(dsp56300_native_long_store_bench PRIVATE dsp56300_native_long_store_fixture)
    add_custom_target(${DSP56300_TOOL_PREFIX}bench-native-long-stores
      COMMAND $<TARGET_FILE:dsp56300_native_long_store_bench> all
      DEPENDS dsp56300_native_long_store_bench USES_TERMINAL VERBATIM)
    add_library(dsp56300_native_partial_store_fixture STATIC test/support/native_partial_store_fixture.cpp)
    target_link_libraries(dsp56300_native_partial_store_fixture PUBLIC dsp56300::core)
    add_executable(dsp56300_native_partial_store_test test/jit/native_partial_store_test.cpp)
    target_link_libraries(dsp56300_native_partial_store_test PRIVATE dsp56300_native_partial_store_fixture)
    add_test(NAME native-partial-stores COMMAND dsp56300_native_partial_store_test)
    add_executable(dsp56300_native_partial_store_bench test/bench/native_partial_store_bench.cpp)
    target_link_libraries(dsp56300_native_partial_store_bench PRIVATE dsp56300_native_partial_store_fixture)
    add_custom_target(${DSP56300_TOOL_PREFIX}bench-native-partial-stores
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
  target_include_directories(dsp56300_cache_storage_test PRIVATE src)
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

  add_executable(dsp56300_deferred_interrupt_test test/core/deferred_interrupt_test.cpp)
  target_link_libraries(dsp56300_deferred_interrupt_test PRIVATE dsp56300::core)
  add_test(NAME deferred-state-interrupt COMMAND dsp56300_deferred_interrupt_test)

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
  add_executable(dsp56300_move_forward_test test/jit/move_forward_test.cpp)
  target_link_libraries(dsp56300_move_forward_test PRIVATE dsp56300::core)
  add_test(NAME native-move-forwarding COMMAND dsp56300_move_forward_test)
  add_executable(dsp56300_window_reuse_test test/jit/window_reuse_test.cpp)
  target_link_libraries(dsp56300_window_reuse_test PRIVATE dsp56300::core)
  add_test(NAME native-window-witnesses COMMAND dsp56300_window_reuse_test)
  add_executable(dsp56300_native_logic_test test/jit/native_logic_test.cpp)
  target_link_libraries(dsp56300_native_logic_test PRIVATE dsp56300::core)
  add_test(NAME native-word-logic COMMAND dsp56300_native_logic_test)
  add_executable(dsp56300_accumulator_result_test test/jit/accumulator_result_test.cpp)
  target_link_libraries(dsp56300_accumulator_result_test PRIVATE dsp56300::core)
  add_test(NAME native-accumulator-results COMMAND dsp56300_accumulator_result_test)
  add_executable(dsp56300_native_dataflow_test test/jit/native_dataflow_test.cpp)
  target_link_libraries(dsp56300_native_dataflow_test PRIVATE dsp56300::core)
  add_test(NAME native-dataflow-boundaries COMMAND dsp56300_native_dataflow_test)
  add_executable(dsp56300_native_loop_test test/jit/native_loop_test.cpp)
  target_link_libraries(dsp56300_native_loop_test PRIVATE dsp56300::core)
  add_test(NAME native-loop-boundaries COMMAND dsp56300_native_loop_test)
  add_executable(dsp56300_native_loop_final_test test/jit/native_loop_final_test.cpp)
  target_link_libraries(dsp56300_native_loop_final_test PRIVATE dsp56300::core)
  add_test(NAME native-loop-final-frames COMMAND dsp56300_native_loop_final_test)
  add_executable(dsp56300_captured_test test/core/captured_test.cpp)
  target_link_libraries(dsp56300_captured_test PRIVATE dsp56300::core)
  add_test(NAME captured-instruction COMMAND dsp56300_captured_test)
  add_executable(dsp56300_arena_test test/jit/arena_test.cpp)
  target_link_libraries(dsp56300_arena_test PRIVATE dsp56300::core)
  if(APPLE)
    add_library(dsp56300_arena_faults SHARED test/support/arena_macos_faults.cpp)
    target_link_libraries(dsp56300_arena_faults PRIVATE dsp56300_quality)
    target_link_libraries(dsp56300_arena_test PRIVATE dsp56300_arena_faults)
  else()
    target_link_options(dsp56300_arena_test PRIVATE
      -Wl,--wrap=memfd_create -Wl,--wrap=ftruncate -Wl,--wrap=mmap)
  endif()
  add_test(NAME native-arena COMMAND dsp56300_arena_test)
  add_executable(dsp56300_native_code_lifetime_test test/jit/native_code_lifetime_test.cpp)
  target_link_libraries(dsp56300_native_code_lifetime_test PRIVATE dsp56300::core)
  add_test(NAME native-code-lifetime COMMAND dsp56300_native_code_lifetime_test)
  add_executable(dsp56300_native_backedge_test test/jit/native_backedge_test.cpp)
  target_link_libraries(dsp56300_native_backedge_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_backedge_test PRIVATE src src/jit)
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

  add_executable(dsp56300_fixture_test test/isa/fixture_test.cpp)
  target_link_libraries(dsp56300_fixture_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_executable(dsp56300_fixture_bench test/bench/fixture_bench.cpp)
  set(dsp56300_numeric_host_source test/bench/numeric_host_asm.cpp)
  set(dsp56300_fir_vector_source test/bench/fir_host_avx2.cpp)
  set(dsp56300_spectral_host_source test/bench/spectral_host_asm.cpp)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
    set(dsp56300_numeric_host_source test/bench/numeric_host_a64.cpp)
    set(dsp56300_fir_vector_source test/bench/fir_host_neon.cpp)
    set(dsp56300_spectral_host_source test/bench/spectral_host_a64.cpp)
  endif()
  add_executable(dsp56300_continuous_bench test/bench/continuous_bench.cpp
    test/bench/fir_stream_host.cpp test/bench/fir_host.cpp ${dsp56300_fir_vector_source}
    test/bench/numeric_stream_host.cpp ${dsp56300_numeric_host_source} test/bench/spectral_stream_host.cpp
    ${dsp56300_spectral_host_source})
  target_compile_options(dsp56300_continuous_bench PRIVATE -O3)
  target_link_libraries(dsp56300_continuous_bench PRIVATE dsp56300::core dsp56300_fixture_format)
  add_executable(dsp56300_numeric_host_test test/bench/numeric_host_test.cpp
    ${dsp56300_numeric_host_source})
  target_link_libraries(dsp56300_numeric_host_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_test(NAME numeric-host-asm COMMAND dsp56300_numeric_host_test)
  add_executable(dsp56300_spectral_host_test test/bench/spectral_host_test.cpp
    ${dsp56300_spectral_host_source})
  target_link_libraries(dsp56300_spectral_host_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_test(NAME spectral-host-asm COMMAND dsp56300_spectral_host_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fft.d56")
  add_test(NAME fir-stream-host-reference COMMAND dsp56300_continuous_bench --verify-fir-host
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir-stream-check.d56")
  foreach(name iir nonlinear spectral)
    add_test(NAME ${name}-stream-host-reference COMMAND dsp56300_continuous_bench --verify-host
      ${name} "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${name}-stream-check.d56")
  endforeach()
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
  add_executable(dsp56300_fir_host_bench test/bench/fir_host_bench.cpp test/bench/fir_host.cpp
    ${dsp56300_fir_vector_source})
  target_link_libraries(dsp56300_fir_host_bench PRIVATE dsp56300::core dsp56300_fixture_format)
  target_compile_options(dsp56300_fir_host_bench PRIVATE -O3)
  add_test(NAME fir-matched-host-asm COMMAND dsp56300_fir_host_bench
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir.d56" --verify)
  add_custom_target(${DSP56300_TOOL_PREFIX}bench-fir-host
    COMMAND $<TARGET_FILE:dsp56300_fir_host_bench> "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir.d56"
    DEPENDS dsp56300_fir_host_bench USES_TERMINAL VERBATIM)
  add_test(NAME fir-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_fir>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/fir-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fir.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/support/reproduce_fixture.cmake")
  target_link_libraries(dsp56300_fixture_bench PRIVATE dsp56300::core dsp56300_fixture_format)
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

  add_test(NAME sm-cpu-domains-fixtures COMMAND dsp56300_fixture_test
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
    get_target_property(core_test_libraries ${core_test_target} LINK_LIBRARIES)
    if(core_test_libraries MATCHES "dsp56300::core" AND
       NOT core_test_libraries MATCHES "dsp56300_codegen_observer")
      target_link_libraries(${core_test_target} PRIVATE dsp56300_codegen_observer)
    endif()
    if(core_test_sources MATCHES "test/")
      target_include_directories(${core_test_target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test/support" "${CMAKE_CURRENT_SOURCE_DIR}/src")
    endif()
  endforeach()
endif()
