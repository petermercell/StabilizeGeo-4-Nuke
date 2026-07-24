cmake_minimum_required(VERSION 3.11)
project(StabilizeGeoUSD)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
#  New 3D / USD-system NDK plugin (Linux). Build against a COMMERCIAL Nuke 17.
#    cmake -B build -DNUKE_VERSION=17.0v1 -DCMAKE_INSTALL_PREFIX=~/.nuke
#    cmake --build build
#    cp build/StabilizeGeoUSD.so ~/.nuke/
# ============================================================================

# Don't set RPATH - rely on Nuke's LD_LIBRARY_PATH environment.
set(CMAKE_SKIP_BUILD_RPATH TRUE)
set(CMAKE_SKIP_INSTALL_RPATH TRUE)

if(NOT DEFINED NUKE_VERSION)
    set(NUKE_VERSION "17.0v1")
endif()
set(NDKDIR /opt/Nuke${NUKE_VERSION})

message(STATUS "Nuke Version: ${NUKE_VERSION}")
message(STATUS "Nuke Directory: ${NDKDIR}")

if(NOT EXISTS ${NDKDIR})
    message(FATAL_ERROR "Nuke directory not found: ${NDKDIR}")
endif()
if(NOT EXISTS ${NDKDIR}/libDDImage.so)
    message(FATAL_ERROR "libDDImage.so not found in: ${NDKDIR}")
endif()
if(NOT EXISTS ${NDKDIR}/include/DDImage/ModifyGeomOp.h)
    message(FATAL_ERROR "New-3D headers not found - need a COMMERCIAL Nuke 17 with the USD NDK.")
endif()

include_directories(${NDKDIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-fPIC)
set(CMAKE_SHARED_LIBRARY_PREFIX "")

add_library(StabilizeGeoUSD SHARED StabilizeGeo.cpp)

# Link DDImage. Any remaining usg/usd symbols resolve from the host (Nuke) at
# load time: on Linux/ELF undefined symbols in a shared object are permitted by
# default (no -z defs), so no dynamic_lookup flag is needed. If you ever want
# the link to flag missing symbols, add -Wl,-z,defs and pull the owning lib
# from ${NDKDIR} here.
target_link_libraries(StabilizeGeoUSD PRIVATE ${NDKDIR}/libDDImage.so)

install(TARGETS StabilizeGeoUSD DESTINATION ${CMAKE_INSTALL_PREFIX})

message(STATUS "========================================")
message(STATUS "  Plugin:       StabilizeGeoUSD.so  (v004)")
message(STATUS "  Nuke Version: ${NUKE_VERSION}")
message(STATUS "  Nuke Dir:     ${NDKDIR}")
message(STATUS "  RPATH:        Not set (portable - uses Nuke's environment)")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "========================================")
