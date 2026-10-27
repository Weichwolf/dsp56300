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
find_program(BLOFELD_CLANG_TIDY NAMES clang-tidy-19 REQUIRED)
find_program(BLOFELD_CLANG_FORMAT NAMES clang-format-19 REQUIRED)

set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

add_library(dsp56300_core STATIC
  src/Core/core.cpp src/Core/core_state.cpp src/Core/arithmetic.cpp src/Core/shifts.cpp
  src/Core/conditional.cpp src/Core/immediate.cpp src/Core/decode.cpp src/Core/moves.cpp
  src/Core/transfers.cpp src/Core/registers.cpp src/Core/agu.cpp src/Core/control.cpp
  src/Core/bits.cpp
  src/Jit/jit.cpp src/Jit/jit_helpers.cpp src/Jit/X64/jit_emitter.cpp src/Jit/X64/x64.cpp
  src/Jit/X64/jit_addition.cpp
  src/Jit/X64/jit_loops.cpp
  src/Jit/native_moves.cpp src/Jit/dataflow.cpp src/Jit/X64/jit_moves.cpp
  src/Jit/X64/jit_move_address.cpp src/Jit/X64/jit_move_register.cpp
  src/Memory/memory_arena.cpp)
add_library(dsp56300::core ALIAS dsp56300_core)
target_include_directories(dsp56300_core PUBLIC include)
target_link_libraries(dsp56300_core PUBLIC blofeld_quality)
target_include_directories(dsp56300_core PRIVATE src/Core src/Jit src/Jit/X64)

file(GLOB_RECURSE DSP56300_OWN_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/*.hpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${DSP56300_OWN_FILES}
  VERBATIM)

if(BUILD_TESTING)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    add_executable(dsp56300_kvm_translation_test test/Kvm/guest.cpp test/Kvm/translation_test.cpp)
    target_link_libraries(dsp56300_kvm_translation_test PRIVATE dsp56300::core)
    target_include_directories(dsp56300_kvm_translation_test PRIVATE
      src/Core src/Jit src/Jit/X64 test)
    add_test(NAME kvm-translation-experiment COMMAND dsp56300_kvm_translation_test)
    set_tests_properties(kvm-translation-experiment PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
  endif()
  add_executable(dsp56300_core_test test/core_test.cpp)
  target_link_libraries(dsp56300_core_test PRIVATE dsp56300::core)
  add_test(NAME core-instructions COMMAND dsp56300_core_test)
  add_executable(dsp56300_jit_test test/jit_test.cpp)
  target_link_libraries(dsp56300_jit_test PRIVATE dsp56300::core)
  add_test(NAME native-boundaries COMMAND dsp56300_jit_test)
  add_executable(dsp56300_native_move_test test/native_move_test.cpp)
  target_link_libraries(dsp56300_native_move_test PRIVATE dsp56300::core)
  add_test(NAME native-move-boundaries COMMAND dsp56300_native_move_test)
  add_executable(dsp56300_native_dataflow_test test/native_dataflow_test.cpp)
  target_link_libraries(dsp56300_native_dataflow_test PRIVATE dsp56300::core)
  add_test(NAME native-dataflow-boundaries COMMAND dsp56300_native_dataflow_test)
  add_executable(dsp56300_native_loop_test test/native_loop_test.cpp)
  target_link_libraries(dsp56300_native_loop_test PRIVATE dsp56300::core)
  add_test(NAME native-loop-boundaries COMMAND dsp56300_native_loop_test)
  add_executable(dsp56300_native_backedge_test test/native_backedge_test.cpp)
  target_link_libraries(dsp56300_native_backedge_test PRIVATE dsp56300::core)
  target_include_directories(dsp56300_native_backedge_test PRIVATE src/Core src/Jit)
  add_test(NAME native-backedge-boundaries COMMAND dsp56300_native_backedge_test)
  add_executable(dsp56300_transfer_test test/transfer_test.cpp)
  target_link_libraries(dsp56300_transfer_test PRIVATE dsp56300::core)
  add_test(NAME transfer-boundaries COMMAND dsp56300_transfer_test)
  add_executable(dsp56300_bit_access_test test/bit_access_test.cpp)
  target_link_libraries(dsp56300_bit_access_test PRIVATE dsp56300::core)
  add_test(NAME bit-access-boundaries COMMAND dsp56300_bit_access_test)
  add_executable(dsp56300_addressing_test test/addressing_test.cpp)
  target_link_libraries(dsp56300_addressing_test PRIVATE dsp56300::core)
  add_test(NAME addressing-boundaries COMMAND dsp56300_addressing_test)
  add_executable(dsp56300_memory_alias_test test/memory_alias_test.cpp)
  target_link_libraries(dsp56300_memory_alias_test PRIVATE dsp56300::core)
  add_test(NAME memory-alias-boundaries COMMAND dsp56300_memory_alias_test)
  add_library(dsp56300_fixture_format STATIC test/binary_fixture.cpp)
  target_include_directories(dsp56300_fixture_format PUBLIC test include)
  target_link_libraries(dsp56300_fixture_format PUBLIC blofeld_quality)
  add_executable(dsp56300_generate_arithmetic test/generate_arithmetic.cpp)
  target_link_libraries(dsp56300_generate_arithmetic PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_bits test/generate_bit_operations.cpp)
  target_link_libraries(dsp56300_generate_bits PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_conditions test/generate_conditions.cpp)
  target_link_libraries(dsp56300_generate_conditions PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_transfers test/generate_transfers.cpp)
  target_link_libraries(dsp56300_generate_transfers PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_bit_access test/generate_bit_access.cpp)
  target_link_libraries(dsp56300_generate_bit_access PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_addressing test/generate_addressing.cpp)
  target_link_libraries(dsp56300_generate_addressing PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_addition test/generate_addition.cpp)
  target_link_libraries(dsp56300_generate_addition PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_fft test/generate_fft.cpp)
  target_link_libraries(dsp56300_generate_fft PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_parallel_moves test/generate_parallel_moves.cpp)
  target_link_libraries(dsp56300_generate_parallel_moves PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_dataflow test/generate_dataflow.cpp)
  target_link_libraries(dsp56300_generate_dataflow PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_generate_loops test/generate_loops.cpp)
  target_link_libraries(dsp56300_generate_loops PRIVATE dsp56300_fixture_format)
  add_executable(dsp56300_fixture_test test/fixture_test.cpp)
  target_link_libraries(dsp56300_fixture_test PRIVATE dsp56300::core dsp56300_fixture_format)
  add_executable(dsp56300_fixture_bench test/fixture_bench.cpp)
  target_link_libraries(dsp56300_fixture_bench PRIVATE dsp56300::core dsp56300_fixture_format)
  find_package(Threads REQUIRED)
  add_executable(dsp56300_fft_native_bench test/fft_native_bench.cpp)
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
  add_test(NAME native-dataflow-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/native-dataflow.d56")
  add_test(NAME loop-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loops.d56")
  add_test(NAME loop-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_loops>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/loops-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/loops.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME native-dataflow-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_dataflow>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/native-dataflow-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/native-dataflow.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME parallel-move-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_parallel_moves>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/parallel-moves-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/parallel-moves.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME fft-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_fft>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/fft-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fft.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME addition-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_addition>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/addition-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addition.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME addressing-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_addressing>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/addressing-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/addressing.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME bit-access-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bit_access>"
    "-DACCESS_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bit-access-regenerated.d56"
    "-DBRANCH_OUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bit-branches-regenerated.d56"
    "-DACCESS_FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-access.d56"
    "-DBRANCH_FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-branches.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_bit_access.cmake")
  add_test(NAME transfer-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_transfers>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/transfers-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/transfers.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME condition-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_conditions>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/conditions-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/conditions.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME bit-operation-fixtures COMMAND dsp56300_fixture_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-operations.d56")
  add_test(NAME bit-fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_bits>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/bits-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/bit-operations.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  add_test(NAME fixture-reproducibility COMMAND "${CMAKE_COMMAND}"
    "-DGENERATOR=$<TARGET_FILE:dsp56300_generate_arithmetic>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/arithmetic-regenerated.d56"
    "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/arithmetic.d56"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/reproduce_fixture.cmake")
  set(DSP56300_PRIVATE_FIXTURES "" CACHE STRING "Semicolon-separated paths to local .d56 fixtures")
  if(DSP56300_PRIVATE_FIXTURES)
    add_test(NAME private-fixtures COMMAND dsp56300_fixture_test ${DSP56300_PRIVATE_FIXTURES})
  endif()
endif()
