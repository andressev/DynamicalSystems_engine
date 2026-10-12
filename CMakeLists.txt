# Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("MathExplorer")

option(BUILD_STANDALONE_SIMS "Build a standalone executable for each sim" ON)

# Vendored libraries that need a build target of our own.
include(cmake/microtex.cmake)

# Engine + sims + apps.
add_subdirectory ("engine")
add_subdirectory ("sims")
add_subdirectory ("apps/explorer")

# Frozen learnOpenGL reference playground (kept building, never modified).
add_subdirectory ("learnOpenGL")
