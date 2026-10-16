cmake_minimum_required(VERSION 3.14)
project(optiflightc_public VERSION 0.1.0 LANGUAGES CXX)

option(OFC_BUILD_TESTS "Build the unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ──── Core library: model, numerics, formulation, use cases ─────
file(GLOB_RECURSE OFC_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM OFC_SOURCES "${PROJECT_SOURCE_DIR}/src/main.cpp")

add_library(ofc_core STATIC ${OFC_SOURCES})
target_include_directories(ofc_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
# Third-party headers as SYSTEM so their warnings do not hide ours.
target_include_directories(ofc_core SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/third_party)
if(MSVC)
    target_compile_options(ofc_core PRIVATE /W4)
else()
    target_compile_options(ofc_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ──── Command-line application ──────────────────────────────────
add_executable(ofc src/main.cpp)
target_link_libraries(ofc PRIVATE ofc_core)

# ──── Tests (no external framework) ─────────────────────────────
if(OFC_BUILD_TESTS)
    enable_testing()
    file(GLOB OFC_TEST_SOURCES CONFIGURE_DEPENDS "tests/*.cpp")
    add_executable(ofc_tests ${OFC_TEST_SOURCES})
    target_link_libraries(ofc_tests PRIVATE ofc_core)
    add_test(NAME ofc_tests COMMAND ofc_tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
