# SPDX-License-Identifier: GPL-3.0-only
# Strict project checks; third-party source keeps its own warning policy.
set(DSP56300_SANITIZER "address-undefined" CACHE STRING "none, address-undefined, or thread")
set_property(CACHE DSP56300_SANITIZER PROPERTY STRINGS none address-undefined thread)
option(DSP56300_FUZZING "Build offline libFuzzer targets" OFF)

include("${CMAKE_CURRENT_LIST_DIR}/Hardening.cmake")
add_library(dsp56300_quality INTERFACE)
target_link_libraries(dsp56300_quality INTERFACE dsp56300_hardening)
target_compile_options(dsp56300_quality INTERFACE
  -Wall -Wextra -Wpedantic -Werror -pedantic-errors
  -Wconversion -Wsign-conversion -Wshadow -Wundef
  -Wcast-align -Wcast-qual -Wdouble-promotion -Wnull-dereference
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wextra-semi -Wvla -Wswitch-enum -Wswitch-default
  -ffp-contract=off -fno-omit-frame-pointer)
# Setup and host API validation use exceptions; ordinary execution reports explicit faults.
target_link_options(dsp56300_quality INTERFACE
  -Wl,--fatal-warnings -Wl,-z,defs)

if(DSP56300_SANITIZER STREQUAL "address-undefined")
  set(dsp56300_sanitizers "address,undefined,float-divide-by-zero,unsigned-integer-overflow,implicit-conversion")
  set(dsp56300_dependency_sanitizers "address,undefined,float-divide-by-zero")
elseif(DSP56300_SANITIZER STREQUAL "thread")
  set(dsp56300_sanitizers "thread")
  set(dsp56300_dependency_sanitizers "thread")
elseif(NOT DSP56300_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unsupported DSP56300_SANITIZER=${DSP56300_SANITIZER}")
endif()
if(DEFINED dsp56300_sanitizers)
  target_compile_options(dsp56300_quality INTERFACE
    "-fsanitize=${dsp56300_sanitizers}" -fno-sanitize-recover=all -g)
  target_link_options(dsp56300_quality INTERFACE "-fsanitize=${dsp56300_sanitizers}")
endif()
if(DSP56300_FUZZING)
  if(NOT DSP56300_SANITIZER STREQUAL "address-undefined")
    message(FATAL_ERROR "Fuzzing requires address-undefined sanitizers")
  endif()
  target_compile_options(dsp56300_quality INTERFACE -fsanitize=fuzzer-no-link)
endif()
