# SPDX-License-Identifier: GPL-3.0-only
# Shared compiler hardening; project diagnostics and sanitizers remain separate.
include(CheckPIESupported)
check_pie_supported(LANGUAGES C CXX OUTPUT_VARIABLE dsp56300_pie_error)
if(NOT CMAKE_C_LINK_PIE_SUPPORTED OR NOT CMAKE_CXX_LINK_PIE_SUPPORTED)
  message(FATAL_ERROR "C and C++ executable PIE support is required: ${dsp56300_pie_error}")
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(CheckCXXSourceCompiles)
check_cxx_source_compiles("#include <cstddef>
#ifndef __GLIBCXX__
#error Not libstdc++
#endif
int main() { return 0; }" DSP56300_USE_LIBSTDCXX)

add_library(dsp56300_hardening INTERFACE)
target_compile_options(dsp56300_hardening INTERFACE
  -fstack-protector-strong
  -Wformat=2 -Werror=format-security $<$<CONFIG:Debug>:-O1>)
# Darwin ASan disables fortified libc calls to preserve sanitizer interception.
if(NOT APPLE OR NOT DSP56300_SANITIZER STREQUAL "address-undefined")
  target_compile_definitions(dsp56300_hardening INTERFACE _FORTIFY_SOURCE=3)
endif()
if(DSP56300_USE_LIBSTDCXX)
  target_compile_definitions(dsp56300_hardening INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:_GLIBCXX_ASSERTIONS>)
endif()
set(DSP56300_CF_PROTECTION OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64|i[3-6]86)$")
  set(DSP56300_CF_PROTECTION ON)
  target_compile_options(dsp56300_hardening INTERFACE -fcf-protection=full)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(dsp56300_hardening INTERFACE -fstack-clash-protection)
  target_link_options(dsp56300_hardening INTERFACE
    -Wl,-z,relro -Wl,-z,now -Wl,-z,noexecstack)
endif()
