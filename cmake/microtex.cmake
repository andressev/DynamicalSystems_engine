# MicroTeX — LaTeX math layout (External/microtex), plus its one dependency tinyxml2.
#
# Builds the core as a static library; engine links it privately for LatexImage, which
# implements the platform side (tex::Graphics2D + font factories, decision 0007). Fonts
# load at runtime from External/microtex/res, see MICROTEX_RES_DIR.

set(MICROTEX_DIR ${CMAKE_SOURCE_DIR}/External/microtex)
set(TINYXML2_DIR ${CMAKE_SOURCE_DIR}/External/tinyxml2)

# every core source; the platform backends (src/platform) are excluded on purpose
file(GLOB_RECURSE MICROTEX_SOURCES CONFIGURE_DEPENDS ${MICROTEX_DIR}/src/*.cpp)
list(FILTER MICROTEX_SOURCES EXCLUDE REGEX "/src/platform/")

add_library(microtex STATIC ${MICROTEX_SOURCES} ${TINYXML2_DIR}/tinyxml2.cpp)

target_include_directories(microtex PUBLIC ${MICROTEX_DIR}/src ${TINYXML2_DIR})

# upstream targets C++17; under C++20 its u8"" literals change type (char8_t)
set_target_properties(microtex PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

if (MSVC)
    target_compile_options(microtex PRIVATE /utf-8 /W0)
endif()

# where a future backend points tex::LaTeX::init()
target_compile_definitions(microtex INTERFACE MICROTEX_RES_DIR="${MICROTEX_DIR}/res")
