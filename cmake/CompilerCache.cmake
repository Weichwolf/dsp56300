# SPDX-License-Identifier: GPL-3.0-only
option(DSP56300_USE_CCACHE "Use ccache when available" ON)

if(DSP56300_USE_CCACHE)
  find_program(DSP56300_CCACHE_EXECUTABLE NAMES ccache)
  if(DSP56300_CCACHE_EXECUTABLE)
    foreach(language C CXX)
      if(NOT DEFINED CMAKE_${language}_COMPILER_LAUNCHER)
        set(CMAKE_${language}_COMPILER_LAUNCHER "${DSP56300_CCACHE_EXECUTABLE}")
      endif()
    endforeach()
  endif()
endif()
