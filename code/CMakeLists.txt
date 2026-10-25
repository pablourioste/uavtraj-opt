cmake_minimum_required(VERSION 3.14)
project(uavtraj_opt VERSION 0.2.0 LANGUAGES CXX)

option(OFC_BUILD_TESTS "Build the unit tests" ON)
set(OFC_IPOPT "AUTO" CACHE STRING "Link the IPOPT solver: AUTO (if found), ON (required) or OFF")
set_property(CACHE OFC_IPOPT PROPERTY STRINGS AUTO ON OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ──── Core library: model, numerics, formulation, solver, pipeline, use cases ─────
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

# ──── Optional: IPOPT ─────
# 1. pkg-config (Linux, macOS, MSYS2), or
# 2. a prebuilt folder with include/, lib/ and bin/ given by IPOPT_DIR (CMake
#    variable or environment variable), e.g. the official Windows release
#    installed by utils/install_ipopt.ps1. Its DLLs are copied next to the
#    executables after each build.
set(OFC_HAVE_IPOPT OFF)
set(OFC_IPOPT_DLLS "")
if(NOT OFC_IPOPT STREQUAL "OFF")
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(IPOPT QUIET IMPORTED_TARGET ipopt)
    endif()
    if(NOT IPOPT_FOUND)
        set(IPOPT_DIR "$ENV{IPOPT_DIR}" CACHE PATH "IPOPT installation folder (include/, lib/, bin/)")
        if(IPOPT_DIR)
            find_path(IPOPT_INCLUDE_DIR IpTNLP.hpp HINTS "${IPOPT_DIR}/include/coin-or" "${IPOPT_DIR}/include/coin"
                      NO_DEFAULT_PATH)
            find_library(IPOPT_LIBRARY NAMES ipopt.dll ipopt HINTS "${IPOPT_DIR}/lib" NO_DEFAULT_PATH)
        endif()
    endif()
    if(IPOPT_FOUND)
        target_link_libraries(ofc_core PUBLIC PkgConfig::IPOPT)
        target_compile_definitions(ofc_core PRIVATE OFC_HAVE_IPOPT)
        set(OFC_HAVE_IPOPT ON)
        message(STATUS "IPOPT ${IPOPT_VERSION}: trajectory optimisation enabled")
    elseif(IPOPT_INCLUDE_DIR AND IPOPT_LIBRARY)
        target_include_directories(ofc_core SYSTEM PUBLIC "${IPOPT_INCLUDE_DIR}")
        target_link_libraries(ofc_core PUBLIC "${IPOPT_LIBRARY}")
        target_compile_definitions(ofc_core PRIVATE OFC_HAVE_IPOPT)
        file(GLOB OFC_IPOPT_DLLS "${IPOPT_DIR}/bin/*.dll")
        set(OFC_HAVE_IPOPT ON)
        message(STATUS "IPOPT from ${IPOPT_DIR}: trajectory optimisation enabled")
    elseif(OFC_IPOPT STREQUAL "ON")
        message(FATAL_ERROR "OFC_IPOPT=ON but IPOPT was not found: install it with pkg-config "
                            "(e.g. coinor-libipopt-dev) or set IPOPT_DIR (Windows: utils/install_ipopt.ps1)")
    else()
        message(STATUS "IPOPT not found: building without the optimiser (use cases UC-08/09 will be skipped)")
    endif()
endif()

# ──── Command-line application ──────────────────────────────────
add_executable(ofc src/main.cpp)
target_link_libraries(ofc PRIVATE ofc_core)
target_compile_definitions(ofc PRIVATE OFC_VERSION="${PROJECT_VERSION}")

# ──── Tests (no external framework) ─────────────────────────────
if(OFC_BUILD_TESTS)
    enable_testing()
    file(GLOB OFC_TEST_SOURCES CONFIGURE_DEPENDS "tests/*.cpp")
    add_executable(ofc_tests ${OFC_TEST_SOURCES})
    target_link_libraries(ofc_tests PRIVATE ofc_core)
    add_test(NAME ofc_tests COMMAND ofc_tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

# ──── Prebuilt IPOPT: its DLLs next to every executable ─────────
if(OFC_IPOPT_DLLS)
    foreach(exe ofc ofc_tests)
        if(TARGET ${exe})
            add_custom_command(TARGET ${exe} POST_BUILD
                               COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OFC_IPOPT_DLLS} $<TARGET_FILE_DIR:${exe}>)
        endif()
    endforeach()
endif()
