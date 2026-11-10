# SPDX-License-Identifier: GPL-3.0-only
# Release already compiles our normal objects with LLVM ThinLTO bitcode.
if(NOT CONFIG STREQUAL "Release")
  message(FATAL_ERROR "Semantic IR export requires the Release preset with ThinLTO")
endif()
