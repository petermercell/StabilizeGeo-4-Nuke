cmake_minimum_required(VERSION 3.11)
project(StabilizeGeo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
#  StabilizeGeo v003 - NDK build (Linux)
#
#  Build against a COMMERCIAL Nuke (app WITHOUT "Non-Commercial" in its name).
#     cmake -B build -DNUKE_VERSION=17.0v1 -DCMAKE_INSTALL_PREFIX=~/.nuke
#     cmake --build build
#     cp build/StabilizeGeo.so ~/.nuke/
# ============================================================================

# Don't set RPATH - rely on Nuke's LD_LIBRARY_PATH environment.
# This makes the plugin portable across different Nuke installations.
set(CMAKE_SKIP_BUILD_RPATH TRUE)
set(CMAKE_SKIP_INSTALL_RPATH TRUE)

if(NOT DEFINED NUKE_VERSION)
    set(NUKE_VERSION "17.0v1")
endif()
set(NDKDIR /opt/Nuke${NUKE_VERSION})

message(STATUS "Nuke Version: ${NUKE_VERSION}")
message(STATUS "Nuke Directory: ${NDKDIR}")

# Verify Nuke directory / NDK presence.
if(NOT EXISTS ${NDKDIR})
    message(FATAL_ERROR "Nuke directory not found: ${NDKDIR}")
endif()
if(NOT EXISTS ${NDKDIR}/libDDImage.so)
    message(FATAL_ERROR "libDDImage.so not found in: ${NDKDIR}")
endif()
if(NOT EXISTS ${NDKDIR}/include/DDImage/Op.h)
    message(FATAL_ERROR "NDK headers not found - point at the COMMERCIAL Nuke install.")
endif()

include_directories(${NDKDIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Uncomment if needed for ABI compatibility with older toolchains:
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
add_compile_options(-fPIC)

set(CMAKE_SHARED_LIBRARY_PREFIX "")

add_library(StabilizeGeo SHARED StabilizeGeo.cpp)

# Link the necessary libraries.
target_link_libraries(StabilizeGeo PRIVATE ${NDKDIR}/libDDImage.so OpenGL)

# No RPATH needed - Nuke's environment provides library paths.
install(TARGETS StabilizeGeo DESTINATION ${CMAKE_INSTALL_PREFIX})

message(STATUS "========================================")
message(STATUS "  Plugin:       StabilizeGeo.so  (v003)")
message(STATUS "  Nuke Version: ${NUKE_VERSION}")
message(STATUS "  Nuke Dir:     ${NDKDIR}")
message(STATUS "  RPATH:        Not set (portable - uses Nuke's environment)")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "========================================")
