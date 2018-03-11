# Abstract-Physics cmake build script.
cmake_minimum_required(VERSION 2.8)
project(APHY)

if(APPLE)
	set(CMAKE_OSX_ARCHITECTURES "x86_64;i386")
endif()

option(APHY_BUILD_SAMPLES "Build APHY Samples" OFF)

# Check the build type
if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Output paths.

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")

# Turn warnings
if (MSVC)
	# using Visual Studio C++
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

if(UNIX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fvisibility=hidden")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fvisibility=hidden")
endif()

# Add some include directories
include_directories(. include)

# Set output dir.
set(EXECUTABLE_OUTPUT_PATH "${APHY_BINARY_DIR}/dist")
set(LIBRARY_OUTPUT_PATH "${APHY_BINARY_DIR}/dist")
option(BUILD_APHY_TEST "Build APHY tests" OFF)

# Find unittest++
find_package(PkgConfig)
if(PKG_CONFIG_FOUND AND NOT APPLE)
    pkg_check_modules(UNITTESTMM unittest++)
    if(UNITTESTMM_FOUND)
        include_directories(${UNITTESTMM_INCLUDE_DIRS})
    endif()
endif()

# Samples libraries
set(APHY_MAIN_LIB APhyBullet)
set(APHY_SAMPLES_LIBS ${APHY_SAMPLES_LIBS} ${SDL2_LIBRARIES} ${APHY_MAIN_LIB})

# Build the implementations
add_subdirectory(implementations)

# Build the samples
if(SDL2_FOUND AND APHY_BUILD_SAMPLES)
	add_subdirectory(samples)
endif()

# Build the tests.
if(UNITTESTMM_FOUND AND BUILD_APHY_TEST)
    add_subdirectory(tests)
endif()
